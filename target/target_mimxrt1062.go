//go:build mimxrt1062

package target

import (
	"device/nxp"
	"errors"
	"machine"
	"math/bits"
	"runtime"

	"github.com/ardnew/teensy40-matrix/hub75"
)

var (
	ErrInvalidFlexIOPins  = errors.New("invalid FlexIO pin configuration")
	ErrInvalidFlexPWMPins = errors.New("invalid FlexPWM pin configuration")
)

var FlexIOHandle = &nxp.FlexIO2

type Board struct {
	flex *nxp.FlexIO
	Pins Pins
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

const (
	// Number of 32-bit FlexIO shifters to use for data buffering.
	// Larger numbers decrease DMA usage.
	// Known working: 1, 2, 3, 4
	DataShifters = 4

	PixelsPerWord = 2
	ShifterPixels = DataShifters * PixelsPerWord

	LatchTimerPrescale = 1 // if min refresh rate is too high, increase this value
	TimerFrequency     = runtime.BUS_FREQ >> LatchTimerPrescale
	NanosecondsPerTick = 1000000000 / TimerFrequency

	// MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT =    10
	// MSB_BLOCK_TICKS_ADJUSTMENT_INCREMENT = (TICKS_PER_ROW / 512)
	// MIN_REFRESH_RATE                     = (((TimerFrequency) / 65535 * (1 << LATCHES_PER_ROW) / ((1 << LATCHES_PER_ROW) - 1) / (MATRIX_SCAN_MOD) / 2) + 1) // cannot refresh slower than this due to PWM register overflow
	// MAX_REFRESH_RATE                     = ((TimerFrequency)/(MIN_BLOCK_PERIOD_TICKS)/(MATRIX_SCAN_MOD)/(LATCHES_PER_ROW) - 1)                              // cannot refresh faster than this due to output bandwidth

	// ROW_CALCULATION_ISR_PRIORITY    = 240 // lowest priority for IMXRT1062
	// ROW_SHIFT_COMPLETE_ISR_PRIORITY = 96  // one step above USB priority
	// TIMER_REGISTERS_TO_UPDATE       = 2

	// size of latch pulse - can be short for V4 shield which doesn't need to update ADDX lines during latch pulse
	// 20 is minimum working value on DP5020B panel
	// set to 100 for improved support with FM6126A panel
	// don't exceed 150 to avoid interference between latch and data transfer
	LatchPulseNanoseconds = 100
	LatchPulseTicks       = LatchPulseNanoseconds / NanosecondsPerTick

	// max delay from rising edge of latch pulse to falling edge of first pixel clock
	// increase this value if DMA use is delaying clock
	// Measured 220 ns delay at 600 MHz clock speed, 160 ns at 816 MHz. Slower speeds are not supported.
	// Using larger delay for safety.
	LatchDelayNanoseconds = 400

	// Pixel clock frequency is generated using the 480 MHz PLL3 clock and a divide-by-n counter. Frequency is independent of CPU clock speed.
	// Must increment divider value by 2 (division ratio is always even)
	// Minimum tested working value is 20 (corresponding to 24 MHz clock frequency) on DP5020B panel
	// Using value of 26 (corresponding to 18.462 MHz clock frequency) to improve stability and reduce glitching
	// Larger values may be more stable, but will decrease maximum refresh rate
	FlexIOClockDivider = 26

	ShiftsPerReload = DataShifters * PixelsPerWord

	// Amount of time required to transfer 32 pixels
	// Adding 200 ns overhead time to improve stability
	MaxTransferNanoseconds = (32*FlexIOClockDivider*1000)/480 + 200
)

func NanosecondsToTicks(nano int64) uint32 {
	return uint32(nano / NanosecondsPerTick)
}

// Padding added to the row data struct to ensure it is divided evenly by the
// FlexIO buffer size plus extra padding for robustness.
func PixelPad(panel hub75.Panel) uint32 {
	return ((-panel.PixelsPerLatch())%ShifterPixels+ShifterPixels)%
		ShifterPixels + ShifterPixels
}

func MinBlockPeriodNanoseconds(panel hub75.Panel) int64 {
	return LatchDelayNanoseconds +
		(MaxTransferNanoseconds*int64(PixelPad(panel)+panel.PixelsPerLatch()))/32
}

func MinBlockPeriodTicks(panel hub75.Panel) uint32 {
	return NanosecondsToTicks(MinBlockPeriodNanoseconds(panel))
}

func TicksPerRow(panel hub75.Panel, refreshRate uint32) uint32 {
	return TimerFrequency / refreshRate / panel.ScanMod()
}

func MSBBlockTicks(panel hub75.Panel, refreshRate uint32) uint32 {
	lpr := panel.LatchesPerRow(refreshRate)
	return (TicksPerRow(panel, refreshRate) / 2) * (1 << lpr) / ((1 << lpr) - 1)
}

func (b *Board) Configure(matrix hub75.Matrix) error {

	b.flex = FlexIOHandle

	if err := b.Pins.Configure(b.flex, matrix.Pins); err != nil {
		return err
	}

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
	b.flex.TIMCMP[0].Set(((ShiftsPerReload*2 - 1) << 8) | (FlexIOClockDivider/2 - 1))

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
	pwm.SM[mod].VAL1.Set(uint16(MinBlockPeriodTicks(matrix.Panel[0])))
	pwm.SM[mod].VAL3.Set(uint16(MinBlockPeriodTicks(matrix.Panel[0])))
	pwm.SM[mod].VAL5.Set(LatchPulseTicks)
	pwm.OUTEN.SetBits((bit << nxp.PWM_OUTEN_PWMA_EN_Pos) & nxp.PWM_OUTEN_PWMA_EN_Msk)
	pwm.OUTEN.SetBits((bit << nxp.PWM_OUTEN_PWMB_EN_Pos) & nxp.PWM_OUTEN_PWMB_EN_Msk)
	pwm.MCTRL.SetBits((bit << nxp.PWM_MCTRL_LDOK_Pos) & nxp.PWM_MCTRL_LDOK_Msk)

	pwm.SM[mod].DMAEN.SetBits(nxp.PWM_SM0DMAEN_VALDE)

	return nil
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
