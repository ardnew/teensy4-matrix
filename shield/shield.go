package shield

import (
	"device/nxp"
	"machine"

	"github.com/ardnew/teensy40-matrix/hub75"
)

// HUB75 — SmartMatrix SmartLED Shield
const (
	HUB75_ADD0_PIN = machine.D6
	HUB75_ADD1_PIN = machine.D9
	HUB75_ADD2_PIN = machine.D10
	HUB75_ADD3_PIN = machine.D12
	HUB75_ADD4_PIN = machine.D11
)

type SmartMatrix struct {
	flex *nxp.FlexIO
	pins hub75.Pins
}

var SmartLED = SmartMatrix{
	flex: &nxp.FlexIO2,
	pins: hub75.Pins{
		OE:  machine.D2,
		LAT: machine.D3,
		CLK: machine.D7,
		R0:  machine.D6,
		G0:  machine.D9,
		B0:  machine.D10,
		R1:  machine.D12,
		G1:  machine.D11,
		B1:  machine.D13,
	},
}

func (s *SmartMatrix) Configure() error {

	// Configure FlexIO pins
	s.pins.CLK.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.R0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.G0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.B0.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.R1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.G1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})
	s.pins.B1.Configure(machine.PinConfig{Mode: machine.PinModeFlexIO})

	// Enable FlexIO peripheral
	s.flex.CTRL.Set(nxp.FLEXIO_CTRL_FLEXEN | nxp.FLEXIO_CTRL_FASTACC)

	shiftCfg := nxp.FlexIOShifterConfig{}
	s.flex.Configure(&s.flex.SHIFTCFG[0], shiftCfg)

	return nil
}
