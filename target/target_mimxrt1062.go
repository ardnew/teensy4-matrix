//go:build mimxrt1062

package target

import (
	"device/nxp"
	"errors"
	"machine"

	"github.com/ardnew/teensy40-matrix/hub75"
)

var ErrInvalidFlexPins = errors.New("invalid FlexIO pin configuration")

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
	CLK uint8
	// Color pins
	R0 uint8
	G0 uint8
	B0 uint8
	R1 uint8
	G1 uint8
	B1 uint8
	// Address pins
	A0 uint8
	A1 uint8
	A2 uint8
	A3 uint8
	A4 uint8
}

func (b *Board) Configure(pins hub75.Pins) error {

	if err := b.Pins.Configure(b.flex, pins); err != nil {
		return err
	}

	b.flex = FlexIOHandle

	// Enable FlexIO peripheral
	b.flex.CTRL.Set(nxp.FLEXIO_CTRL_FLEXEN | nxp.FLEXIO_CTRL_FASTACC)

	shiftCfg := nxp.FlexIOShifterConfig{}
	b.flex.Configure(&b.flex.SHIFTCFG[0], shiftCfg)

	return nil
}

func (p *Pins) Configure(bus *nxp.FlexIO, pins hub75.Pins) error {
	p.Pins = pins

	// Configure FlexIO pins
	p.CLK.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.R0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.G0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.B0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.R1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.G1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	p.B1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})

	p.flex.CLK = p.CLK.FlexIO(bus)
	p.flex.R0 = p.R0.FlexIO(bus)
	p.flex.G0 = p.G0.FlexIO(bus)
	p.flex.B0 = p.B0.FlexIO(bus)
	p.flex.R1 = p.R1.FlexIO(bus)
	p.flex.G1 = p.G1.FlexIO(bus)
	p.flex.B1 = p.B1.FlexIO(bus)

	lo := min(p.flex.R0, min(p.flex.G0, min(p.flex.B0, min(p.flex.R1, min(p.flex.G1, p.flex.B1)))))
	hi := max(p.flex.R0, max(p.flex.G0, max(p.flex.B0, max(p.flex.R1, max(p.flex.G1, p.flex.B1)))))

	ob := 0
	if hi < lo+16 {
		ob = 1
	}
	lb := 0
	if p.flex.CLK < lo {
		lb = 1
	}
	hb := 0
	if p.flex.CLK > hi {
		hb = 1
	}

	if 0 != ob&(lb|hb) {
		return ErrInvalidFlexPins
	}

	p.flex.R0 -= lo
	p.flex.G0 -= lo
	p.flex.B0 -= lo
	p.flex.R1 -= lo
	p.flex.G1 -= lo
	p.flex.B1 -= lo

	p.flex.A0 = p.A0.FlexIO(bus) - lo
	p.flex.A1 = p.A1.FlexIO(bus) - lo
	p.flex.A2 = p.A2.FlexIO(bus) - lo
	p.flex.A3 = p.A3.FlexIO(bus) - lo
	p.flex.A4 = p.A4.FlexIO(bus) - lo

	return nil
}

func min(a, b uint8) uint8 {
	if a < b {
		return a
	}
	return b
}

func max(a, b uint8) uint8 {
	if a > b {
		return a
	}
	return b
}
