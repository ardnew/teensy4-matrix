//go:build mimxrt1062

package target

import (
	"device/nxp"
	"errors"
	"machine"
	"math/bits"
	"runtime"
	"runtime/volatile"
	"unsafe"

	"github.com/ardnew/teensy40-matrix/color"
	"github.com/ardnew/teensy40-matrix/hub75"
	"github.com/ardnew/teensy40-matrix/queue"
)

var (
	ErrInvalidFlexIOPins  = errors.New("invalid FlexIO pin configuration")
	ErrInvalidFlexPWMPins = errors.New("invalid FlexPWM pin configuration")

	ErrMaxRefreshRate = errors.New("refresh rate greater than configuration maximum")
	ErrMinRefreshRate = errors.New("refresh rate less than configuration minimum")
)

var FlexIOHandle = &nxp.FlexIO2

type Board struct {
	Panel []hub75.Panel
	Pins  Pins

	flex *nxp.FlexIO

	dmaUpdate *nxp.DMAChannel
	dmaEnable *nxp.DMAChannel
	dmaOutput *nxp.DMAChannel

	ring     queue.Ring
	underrun volatile.Register8
	current  volatile.Register32

	dim uint32
}

type Pins struct {
	hub75.Pins
	flex flexPins
}

type flexPins struct {
	// Signal pins
	OEN nxp.FlexPWMPin
	LAT nxp.FlexPWMPin
	CLK nxp.FlexIOPin
	// Data pins
	R0 nxp.FlexIOPin
	G0 nxp.FlexIOPin
	B0 nxp.FlexIOPin
	R1 nxp.FlexIOPin
	G1 nxp.FlexIOPin
	B1 nxp.FlexIOPin
	// Data pins span
	Lo nxp.FlexIOPin
	Hi nxp.FlexIOPin
	// Address pins
	A0 nxp.FlexIOPin
	A1 nxp.FlexIOPin
	A2 nxp.FlexIOPin
	A3 nxp.FlexIOPin
	A4 nxp.FlexIOPin
}

type timerControl struct {
	oen, per volatile.Register16
}

type rowBits struct {
	px      [hub75.DefaultPanelWidth]uint16
	address uint32
	timer   timerControl
}

type rowPlane struct{ plane [hub75.LatchesPerRow]rowBits }

type rgb color.Rgb48

type frameBuffer [][]rgb

// Buffer size constants
const (
	// Number of 32-bit FlexIO shifters to use for data buffering.
	// Larger numbers decrease DMA usage.
	// Known working: 1, 2, 3, 4
	DataShifters = 2

	NumRowBuffers = 4

	PixelsPerWord = 2
	ShifterPixels = DataShifters * PixelsPerWord
	// ShiftsPerLoad = DataShifters * PixelsPerWord

	// NumDMABufferRowBytes = hub75.LatchesPerRow * hub75.DefaultPanelWidth * 2
)

const (
	RowShiftInterruptPriority       = 0x60
	RowCalculationInterruptPriority = 0xF0
)

// Timing constants
const (
	LatchTimerPrescale = 1 // Increase this value if min refresh rate is too high
	TimerFrequencyHz   = runtime.BUS_FREQ >> LatchTimerPrescale
	NanosPerTick       = 1000000000 / TimerFrequencyHz

	// MIN_REFRESH_RATE                     = (((TimerFrequencyHz) / 65535 * (1 << LATCHES_PER_ROW) / ((1 << LATCHES_PER_ROW) - 1) / (MATRIX_SCAN_MOD) / 2) + 1) // cannot refresh slower than this due to PWM register overflow
	// MAX_REFRESH_RATE                     = ((TimerFrequencyHz)/(MIN_BLOCK_PERIOD_TICKS)/(MATRIX_SCAN_MOD)/(LATCHES_PER_ROW) - 1)                              // cannot refresh faster than this due to output bandwidth

	// Size of latch pulse - can be short for V4 shield which doesn't need to
	// update ADDX lines during latch pulse.
	// 20 is minimum working value on DP5020B panel
	// Set to 100 for improved support with FM6126A panel.
	// Don't exceed 150 to avoid interference between latch and data transfer.
	LatchPulseNanos = 100
	LatchPulseTicks = LatchPulseNanos / NanosPerTick

	// Max delay from falling edge of latch pulse to rising edge of first pixel
	// clock.
	// Increase this value if DMA use is delaying clock.
	// Measured 220 ns delay at 600 MHz clock speed, 160 ns at 816 MHz. Slower
	// speeds are not supported.
	LatchClockDelayNanos = 400 // Using larger delay for safety.

	// Pixel clock frequency is generated using the 480 MHz PLL3 clock and a
	// divide-by-n counter. Frequency is independent of CPU clock speed.
	// Must increment divider value by 2 (division ratio is always even).
	// Minimum tested working value is 20 (corresponding to 24 MHz clock
	// frequency) on DP5020B panel.
	// Using value of 26 (corresponding to 18.462 MHz clock frequency) to improve
	// stability and reduce glitching.
	// Larger values may be more stable, but will decrease maximum refresh rate.
	FlexIOClockDivider = 26

	// Amount of time required to transfer 32 pixels.
	// Adding 200 ns overhead time to improve stability.
	MaxTransferNanos = (32*FlexIOClockDivider*1000)/480 + 200
)

func nanosToTicks(nano int64) uint32 { return uint32(nano / NanosPerTick) }

// var rowDataBuffer [NumDMABufferRowBytes * NumRowBuffers / uint32(unsafe.Sizeof(uint32(0)))]uint32

//go:section .dmabuffer
//go:align 32
var rowData [NumRowBuffers]rowPlane

//go:section .dmabuffer
var timerIdle timerControl

//go:section .dmabuffer
var timerLUT [hub75.LatchesPerRow]timerControl

//go:section .dmabuffer
var enablerSourceByte volatile.Register8

var (
	fb  []frameBuffer
	rb1 []rgb
	rb2 []rgb
)

func (b *Board) Configure(matrix hub75.Matrix) error {

	b.Panel = matrix.Panel
	b.flex = FlexIOHandle

	if err := b.Pins.Configure(b.flex, matrix.Pins); err != nil {
		return err
	}

	b.ring.Init(uint32(len(rowData)))

	timerIdle.per.Set(uint16(b.minBlockPeriodTicks()))
	timerIdle.oen.Set(timerIdle.per.Get() + 1)
	nxp.FlushDcache(uintptr(unsafe.Pointer(&timerIdle)), unsafe.Sizeof(timerIdle))

	if err := b.updateTimerLookupTable(); err != nil {
		return err
	}

	fb = make([]frameBuffer, 2)
	for s := range fb {
		fb[s] = make([][]rgb, b.height())
		for y := range fb[s] {
			fb[s][y] = make([]rgb, b.width())
			for x := range fb[s][y] {
				fb[s][y][x].R = 0xFFFF
			}
		}
	}

	rbLen := b.pixelsPerRefresh()
	rb1 = make([]rgb, rbLen)
	rb2 = make([]rgb, rbLen)

	b.handleRowCalculation()

	// Enable FlexIO peripheral
	b.flex.CTRL.Set(nxp.FLEXIO_CTRL_FLEXEN | nxp.FLEXIO_CTRL_FASTACC)

	// Shifter[0 .. DataShifters] config
	shifterConfig := nxp.FlexIOShifterConfig{
		Width:    uint32(b.Pins.flex.Hi.Pin - b.Pins.flex.Lo.Pin),
		Input:    1, // next shifter
		StopBit:  0, // disabled
		StartBit: 0, // disabled, load on enable
	}

	// Shifter[0] config
	if err := b.flex.SetShifterConfig(0, shifterConfig); err != nil {
		return err
	}

	// Shifter[0] control
	if err := b.flex.SetShifterControl(0,
		nxp.FlexIOShifterControl{
			TimSel: 0, // timer 0
			TimPol: 1, // falling edge
			PinCfg: 3, // shifter pin output
			PinSel: uint32(b.Pins.flex.Lo.Pin),
			PinPol: 0, // active high
			Mode:   2, // transmit, load on timer expiration
		}); err != nil {
		return err
	}

	// Shifter[1 .. DataShifters]:
	for i := 1; i <= DataShifters; i++ {

		// Shifter[N] config
		if err := b.flex.SetShifterConfig(i, shifterConfig); err != nil {
			return err
		}

		// Shifter[N] control
		if err := b.flex.SetShifterControl(i,
			nxp.FlexIOShifterControl{
				TimSel: 0, // timer 0
				TimPol: 1, // falling edge
				PinCfg: 0, // disabled
				Mode:   2, // transmit, load on timer expiration
			}); err != nil {
			return err
		}
	}

	// Timer[0] config
	if err := b.flex.SetTimerConfig(0,
		nxp.FlexIOTimerConfig{
			Output:    1, // logic 0 when enabled, unaffected by timer reset
			Decrement: 0, // on FlexIO clock, shift clock equals timer output
			Reset:     0, // never
			Disable:   2, // on timer compare
			Enable:    2, // on trigger assert
			Stop:      0, // disabled
			Start:     0, // disabled
		}); err != nil {
		return err
	}

	// Timer[0] control
	if err := b.flex.SetTimerControl(0,
		nxp.FlexIOTimerControl{
			TrgSel: 1 + 4*(DataShifters-1),
			TrgPol: 1, // active low
			TrgSrc: 1, // internal trigger
			PinCfg: 3, // timer pin output
			PinSel: uint32(b.Pins.flex.CLK.Pin),
			PinPol: 0, // active high
			Mode:   1, // dual 8-bit counters baud
		}); err != nil {
		return err
	}

	// Configure timer compare register:
	//  - Lower 8 bits configure the FlexIO clock divide ratio to generate the
	//    pixel clock
	//  - Upper 8 bits configure the number of pixel clock cycles to be generated
	//    each time the shifters are reloaded
	b.flex.TIMCMP[0].Set(((ShifterPixels*2 - 1) << 8) | (FlexIOClockDivider/2 - 1))

	// Enable DMA trigger when data is loaded into the last data shifter so that
	// reloading will occur automatically
	b.flex.SHIFTSDEN.SetBits(1 << (DataShifters - 1))

	pwm := b.Pins.flex.LAT.Bus
	mod := b.Pins.flex.LAT.Sub
	bit := uint16(1) << mod

	pwm.MCTRL.ClearBits((bit << nxp.PWM_MCTRL_RUN_Pos) & nxp.PWM_MCTRL_RUN_Msk)
	pwm.MCTRL.SetBits((bit << nxp.PWM_MCTRL_CLDOK_Pos) & nxp.PWM_MCTRL_CLDOK_Msk)
	pwm.SM[mod].CTRL.Set(nxp.PWM_SM0CTRL_FULL |
		((LatchTimerPrescale << nxp.PWM_SM0CTRL_PRSC_Pos) & nxp.PWM_SM0CTRL_PRSC_Msk))
	pwm.SM[mod].VAL1.Set(65535 - 1)
	pwm.SM[mod].VAL3.Set(32767)
	pwm.SM[mod].VAL5.Set(LatchPulseTicks)
	pwm.OUTEN.SetBits((bit << nxp.PWM_OUTEN_PWMA_EN_Pos) & nxp.PWM_OUTEN_PWMA_EN_Msk)
	pwm.OUTEN.SetBits((bit << nxp.PWM_OUTEN_PWMB_EN_Pos) & nxp.PWM_OUTEN_PWMB_EN_Msk)
	pwm.MCTRL.SetBits((bit << nxp.PWM_MCTRL_LDOK_Pos) & nxp.PWM_MCTRL_LDOK_Msk)

	pwm.SM[mod].DMAEN.SetBits(nxp.PWM_SM0DMAEN_VALDE)

	var err error
	if b.dmaUpdate, err = nxp.AllocDMA(); err != nil {
		return err
	}
	if b.dmaEnable, err = nxp.AllocDMA(); err != nil {
		return err
	}
	if b.dmaOutput, err = nxp.AllocDMA(); err != nil {
		return err
	}

	b.dmaUpdate.Enable(false)
	b.dmaEnable.Enable(false)
	b.dmaOutput.Enable(false)

	{
		const (
			minorBytes = unsafe.Sizeof(timerControl{})
			majorIters = 1 // uintptr(hub75.LatchesPerRow)

			srcOff  = unsafe.Sizeof(rowData[0].plane[0].timer.oen)
			srcLast = unsafe.Sizeof(rowData[0].plane[0]) - minorBytes
		)

		srcAddr := uintptr(unsafe.Pointer(&(rowData[0].plane[0].timer.oen)))
		dstAddr := uintptr(unsafe.Pointer(&(pwm.SM[mod].VAL3)))
		dstOff := uintptr(unsafe.Pointer(&(pwm.SM[mod].VAL1))) - dstAddr
		dstLast := negptr(2 * dstOff)
		// minorOff := dstLast

		b.dmaUpdate.TCD.SADDR.Set(uint32(srcAddr))
		b.dmaUpdate.TCD.SOFF.Set(uint16(srcOff))
		b.dmaUpdate.TCD.SLAST.Set(uint32(srcLast))
		b.dmaUpdate.TCD.ATTR.Set( // 16-bit source (MSB) and destination (LSB)
			(nxp.DMATransferSize2Bytes << 8) | nxp.DMATransferSize2Bytes)
		b.dmaUpdate.TCD.NBYTES.Set(uint32(minorBytes))
		b.dmaUpdate.TCD.DADDR.Set(uint32(dstAddr))
		b.dmaUpdate.TCD.DOFF.Set(uint16(dstOff))
		b.dmaUpdate.TCD.DLAST.Set(uint32(dstLast))
		b.dmaUpdate.TCD.BITER.Set(uint16(majorIters))
		b.dmaUpdate.TCD.CITER.Set(uint16(majorIters))
		_, latTrigger := b.Pins.flex.LAT.DMAChannel()
		b.dmaUpdate.SetHardwareTrigger(latTrigger)
		b.dmaUpdate.SetInterruptOnSoftware(b.handleRowCalculation)
		b.dmaUpdate.SetInterruptPriority(RowCalculationInterruptPriority)
	}

	{
		enablerSourceByte.Set((uint8(b.dmaOutput.ID()) << nxp.DMA_SERQ_SERQ_Pos) &
			nxp.DMA_SERQ_SERQ_Msk)
		nxp.FlushDcache(uintptr(unsafe.Pointer(&enablerSourceByte)),
			unsafe.Sizeof(enablerSourceByte))
		b.dmaEnable.SetSource8(&(enablerSourceByte))
		b.dmaEnable.SetDestination8(&(b.dmaEnable.SERQ))
		b.dmaEnable.SetTransferCount(hub75.LatchesPerRow)
		b.dmaEnable.SetTriggerOnCompletion(b.dmaUpdate.ID())
	}

	{
		const (
			minorIters = uintptr(DataShifters)
			minorBytes = minorIters * unsafe.Sizeof(uint32(0))
			majorBytes = unsafe.Sizeof(rowData[0].plane[0].px)
			majorIters = majorBytes / minorBytes

			srcOff  = unsafe.Sizeof(uint32(0))
			srcLast = unsafe.Sizeof(rowData[0].plane[0]) - uintptr(majorBytes)
			dstOff  = unsafe.Sizeof(uint32(0))
		)

		srcAddr := uintptr(unsafe.Pointer(&(rowData[0].plane[0].px[0])))

		dstAddr := uintptr(unsafe.Pointer(&(b.flex.SHIFTBUF[0])))
		dstLast := negptr(minorIters * dstOff)
		minorOff := dstLast

		b.dmaOutput.TCD.SADDR.Set(uint32(srcAddr))
		b.dmaOutput.TCD.SOFF.Set(uint16(srcOff))
		b.dmaOutput.TCD.SLAST.Set(uint32(srcLast))
		b.dmaOutput.TCD.ATTR.Set( // 32-bit source (MSB) and destination (LSB)
			(nxp.DMATransferSize4Bytes << 8) | nxp.DMATransferSize4Bytes)
		b.dmaOutput.TCD.NBYTES.Set((uint32(1) << 30) |
			((uint32(minorOff) & 0xFFFFF) << 10) | (uint32(minorBytes) & 0x3FF))
		b.dmaOutput.TCD.DADDR.Set(uint32(dstAddr))
		b.dmaOutput.TCD.DOFF.Set(uint16(dstOff))
		b.dmaOutput.TCD.DLAST.Set(uint32(dstLast))
		b.dmaOutput.TCD.BITER.Set(uint16(majorIters))
		b.dmaOutput.TCD.CITER.Set(uint16(majorIters))
		b.dmaOutput.SetDisableOnComplete(true)
		b.dmaOutput.SetHardwareTrigger(b.flex.DMAChannel(DataShifters - 1))
		b.dmaOutput.SetInterruptOnComplete(b.handleRowShiftComplete)
		b.dmaOutput.SetInterruptPriority(RowShiftInterruptPriority)
	}

	b.dmaEnable.Enable(true)
	b.dmaUpdate.Enable(true)

	b.setRowAddress(b.ring.NextRead())

	pwm.MCTRL.SetBits((bit << nxp.PWM_MCTRL_RUN_Pos) & nxp.PWM_MCTRL_RUN_Msk)

	return nil
}

func (b *Board) updateTimerLookupTable() error {

	const lpr = 1 << hub75.LatchesPerRow
	tpr := b.ticksPerRow()
	min := b.minBlockPeriodTicks()

	if min*hub75.LatchesPerRow >= tpr {
		return ErrMaxRefreshRate
	}
	if hub75.RefreshRateHz < b.minRefreshRateHz() {
		return ErrMinRefreshRate
	}

	inc := tpr / 512
	blk := tpr/2*lpr/(lpr-1) + inc

	for {
		blk -= inc
		tik := uint32(0)
		for i := 0; i < hub75.LatchesPerRow; i++ {
			k := (blk >> (hub75.LatchesPerRow - i - 1)) + LatchPulseTicks
			if k < min {
				k = min
			}
			tik += k
		}
		if tik <= tpr {
			break
		}
	}

	for i := 0; i < hub75.LatchesPerRow; i++ {
		k := uint32(blk) >> (hub75.LatchesPerRow - i - 1)
		per := k + LatchPulseTicks
		oen := k*b.dim/hub75.DimmingMaximum + LatchPulseTicks
		if per < min {
			d := min - per
			per += d
			oen += d
		}
		timerLUT[i].per.Set(uint16(per))
		timerLUT[i].oen.Set(uint16(oen))
	}
	nxp.FlushDeleteDcache(
		uintptr(unsafe.Pointer(&(timerLUT[0]))), unsafe.Sizeof(timerLUT))

	return nil
}

func (b *Board) setRowAddress(row uint32) {
	if int(row) < len(rowData) {
		current := rowData[row].plane[0].address
		address := uint32(0)
		if (current & 0x01) != 0 {
			address |= 1 << b.Pins.flex.A0.Pin
		}
		if (current & 0x02) != 0 {
			address |= 1 << b.Pins.flex.A1.Pin
		}
		if (current & 0x04) != 0 {
			address |= 1 << b.Pins.flex.A2.Pin
		}
		if (current & 0x08) != 0 {
			address |= 1 << b.Pins.flex.A3.Pin
		}
		if (current & 0x10) != 0 {
			address |= 1 << b.Pins.flex.A4.Pin
		}
		b.flex.SHIFTBUF[DataShifters].Set(address)
	}
}

// handleRowShiftComplete runs at the completion of dmaOutput when a complete
// bitplane has been output to the panel.
//
// If we have finished all the bitplanes for this row, it's time to update the
// source address of dmaOutput to point to the next row in rowData.
// Otherwise, the interrupt does nothing.
//
// To determine whether all the bitplanes are done, we look at dmaEnable, which
// keeps an iteration count that corresponds to the bitplanes.
//   - dmaEnable is serviced first each cycle, before handleRowShiftComplete.
//   - On the last bitplane, CITER (current iteration) = 1.
//   - When transaction finishes, CITER is reset to BITER (initial iteration).
//   - So if CITER == BITER, that means the final bitplane was just completed.
//     -- In this case it's time to update dmaOutput to the next row.
func (b *Board) handleRowShiftComplete() {

	b.dmaOutput.ClearInterrupt()

	if b.dmaEnable.TCD.CITER.Get() == b.dmaEnable.TCD.BITER.Get() {
		if b.ring.Read(); b.ring.Empty() {
			// repeatedly load from values that set period to minBlockPeriodTicks and
			// disable OE
			b.dmaUpdate.TCD.SADDR.Set(uint32(uintptr(unsafe.Pointer(&timerIdle))))
			// repeat idle timer
			b.dmaUpdate.TCD.SLAST.Set(uint32(negptr(unsafe.Sizeof(timerIdle))))
			// don't trigger dmaOutput until buffer is ready
			b.dmaUpdate.TCD.CSR.ClearBits(nxp.DMA_TCD0_CSR_MAJORELINK)
			b.underrun.Set(1)
		} else {
			row := b.ring.NextRead()
			b.dmaOutput.TCD.SADDR.Set(
				uint32(uintptr(unsafe.Pointer(&(rowData[row].plane[0].px[0])))))
			b.dmaUpdate.TCD.SADDR.Set(
				uint32(uintptr(unsafe.Pointer(&(rowData[row].plane[0].timer.oen)))))
			b.setRowAddress(row)
		}
		b.dmaUpdate.SetInterruptPending(true)
	}
}

// handleRowCalculation refills the row buffer.
// It may be interrupted by handleRowShiftComplete.
func (b *Board) handleRowCalculation() {
	mod := b.scanMod()
	for !b.ring.Full() {
		row := b.current.Get()
		b.loadBuffers(row)
		b.writeBuffers()
		if row += 1; row >= mod {
			row = 0
		}
		b.current.Set(row)
	}
}

func (b *Board) loadBuffers(row uint32) {

	b.resetRowBuffer(rb1)
	b.resetRowBuffer(rb2)

	buf := &(rowData[b.ring.NextWrite()])

	var y0, y1 uint32
	for i := range b.Panel {
		y0 = row + uint32(len(b.Panel)-i-1)*b.height()
		y1 = y0 + b.scanMod()

		dy := i * int(b.width())
		for j := 0; j < int(b.width()); j++ {
			rb1[dy+j] = fb[0][y0][j]
			rb2[dy+j] = fb[0][y1][j]
		}
	}

	for i := uint32(0); i < b.pixelsPerRefresh(); {
		sz := b.width()
		for k := uint32(0); k < sz; k++ {
			pos := i + k
			r0 := rb1[pos].R
			g0 := rb1[pos].G
			b0 := rb1[pos].B
			r1 := rb2[pos].R
			g1 := rb2[pos].G
			b1 := rb2[pos].B

			var data uint16
			shl := 16 - hub75.ColorDepth
			msk := uint16(1) << shl
			for bit := 0; bit < hub75.LatchesPerRow; bit++ {
				data = (r0 & msk) << b.Pins.flex.R0.Pin
				data |= (g0 & msk) << b.Pins.flex.G0.Pin
				data |= (b0 & msk) << b.Pins.flex.B0.Pin
				data |= (r1 & msk) << b.Pins.flex.R1.Pin
				data |= (g1 & msk) << b.Pins.flex.G1.Pin
				data |= (b1 & msk) << b.Pins.flex.B1.Pin
				data >>= shl
				shl++
				msk <<= 1
				buf.plane[bit].px[pos] = data
			}
		}
		i += sz
	}
	buf.plane[0].address = row
}

func (b *Board) writeBuffers() {
	buf := &(rowData[b.ring.NextWrite()])
	for i := 0; i < hub75.LatchesPerRow; i++ {
		buf.plane[i].timer.per.Set(timerLUT[i].per.Get())
		buf.plane[i].timer.oen.Set(timerLUT[i].oen.Get())
	}
	nxp.FlushDcache(uintptr(unsafe.Pointer(buf)), unsafe.Sizeof(*buf))
	b.ring.Write()
}

func (b *Board) width() uint32 {
	return b.Panel[0].Width()
}

func (b *Board) height() uint32 {
	return b.Panel[0].Height()
}

func (b *Board) scanMod() uint32 {
	return b.Panel[0].ScanMod()
}

func (b *Board) bytesPerRow() uint32 {
	return b.Panel[0].BufferBytesPerRow()
}

func (b *Board) ticksPerRow() uint32 {
	return b.Panel[0].TicksPerRow(TimerFrequencyHz)
}

func (b *Board) minBlockPeriodTicks() uint32 {
	return nanosToTicks(LatchClockDelayNanos +
		(MaxTransferNanos*int64(b.Panel[0].PixelsPerLatch()))/32)
}

func (b *Board) minRefreshRateHz() uint32 {
	return b.Panel[0].MinRefreshRateHz(TimerFrequencyHz)
}

func (b *Board) pixelsPerRefresh() uint32 {
	return b.Panel[0].PixelsPerLatch() / b.Panel[0].PhysicalRowsPerRefresh()
}

func (b *Board) resetRowBuffer(fb []rgb) {
	for i := range fb {
		fb[i].R, fb[i].G, fb[i].B = 0, 0, 0
	}
}

func (p *Pins) Configure(bus *nxp.FlexIO, pins hub75.Pins) error {
	p.Pins = pins

	// Configure PWM pins
	p.OEN.Configure(machine.PinConfig{Mode: machine.PinModeFlexPWM})
	p.LAT.Configure(machine.PinConfig{Mode: machine.PinModeFlexPWM})

	var okLAT, okOEN bool
	p.flex.LAT, okLAT = p.LAT.FlexPWM()
	p.flex.OEN, okOEN = p.OEN.FlexPWM()
	if !okLAT || !okOEN {
		return ErrInvalidFlexPWMPins
	}

	// Configure FlexIO pins
	p.CLK.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.R0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.G0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.B0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.R1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.G1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.B1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})

	instance := bits.TrailingZeros32(uint32(bus.ID())) + 1

	var okCLK, okR0, okG0, okB0, okR1, okG1, okB1 bool
	p.flex.CLK, okCLK = p.CLK.FlexIO(instance)
	p.flex.R0, okR0 = p.R0.FlexIO(instance)
	p.flex.G0, okG0 = p.G0.FlexIO(instance)
	p.flex.B0, okB0 = p.B0.FlexIO(instance)
	p.flex.R1, okR1 = p.R1.FlexIO(instance)
	p.flex.G1, okG1 = p.G1.FlexIO(instance)
	p.flex.B1, okB1 = p.B1.FlexIO(instance)
	if !okCLK || !okR0 || !okG0 || !okB0 || !okR1 || !okG1 || !okB1 {
		return ErrInvalidFlexIOPins
	}

	// In either case, (x < y) or (x >= y), it holds that min(x, y) <= max(x, y).
	// So we know that lo <= hi.
	p.flex.Lo =
		min(p.flex.R0, min(p.flex.G0, min(p.flex.B0,
			min(p.flex.R1, min(p.flex.G1, p.flex.B1)))))
	p.flex.Hi =
		max(add(p.flex.Lo, +8),
			max(p.flex.R0, max(p.flex.G0, max(p.flex.B0,
				max(p.flex.R1, max(p.flex.G1, p.flex.B1))))))

	// All data pins must be within a 16-bit span.
	if p.flex.Hi.Pin-p.flex.Lo.Pin >= 16 ||
		// And CLK must be outside of the 16-bit data span.
		(p.flex.CLK.Pin >= p.flex.Lo.Pin && p.flex.CLK.Pin <= p.flex.Hi.Pin) {
		return ErrInvalidFlexIOPins
	}

	p.flex.R0 = add(p.flex.R0, -int(p.flex.Lo.Pin))
	p.flex.G0 = add(p.flex.G0, -int(p.flex.Lo.Pin))
	p.flex.B0 = add(p.flex.B0, -int(p.flex.Lo.Pin))
	p.flex.R1 = add(p.flex.R1, -int(p.flex.Lo.Pin))
	p.flex.G1 = add(p.flex.G1, -int(p.flex.Lo.Pin))
	p.flex.B1 = add(p.flex.B1, -int(p.flex.Lo.Pin))

	var okA0, okA1, okA2, okA3, okA4 bool
	p.flex.A0, okA0 = p.A0.FlexIO(instance)
	p.flex.A1, okA1 = p.A1.FlexIO(instance)
	p.flex.A2, okA2 = p.A2.FlexIO(instance)
	p.flex.A3, okA3 = p.A3.FlexIO(instance)
	p.flex.A4, okA4 = p.A4.FlexIO(instance)
	if !okA0 || !okA1 || !okA2 || !okA3 || !okA4 {
		return ErrInvalidFlexIOPins
	}

	p.flex.A0 = add(p.flex.A0, -int(p.flex.Lo.Pin))
	p.flex.A1 = add(p.flex.A1, -int(p.flex.Lo.Pin))
	p.flex.A2 = add(p.flex.A2, -int(p.flex.Lo.Pin))
	p.flex.A3 = add(p.flex.A3, -int(p.flex.Lo.Pin))
	p.flex.A4 = add(p.flex.A4, -int(p.flex.Lo.Pin))

	return nil
}

func add(p nxp.FlexIOPin, x int) nxp.FlexIOPin {
	p.Pin = uint8(int(p.Pin) + x)
	return p
}

func min(a, b nxp.FlexIOPin) nxp.FlexIOPin {
	if a.Pin < b.Pin {
		return a
	}
	return b
}

func max(a, b nxp.FlexIOPin) nxp.FlexIOPin {
	if a.Pin > b.Pin {
		return a
	}
	return b
}

// negptr returns an unsigned pointer value whose bit representation is exactly
// equal to the negative of the given value (in two's complement notation).
// For example, negptr(12345) = 4294954951 because both int32(-12345) and
// uint32(4294954951) have the same 32-bit representation.
// This routine is useful in situations where unsigned 32-bit data types are
// required to hold signed 32-bit values, such as in memory-mapped registers.
func negptr(x uintptr) uintptr { return ^x + 1 }
