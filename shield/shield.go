package shield

import (
	"machine"

	"github.com/ardnew/teensy40-matrix/hub75"
	"github.com/ardnew/teensy40-matrix/target"
)

type SmartMatrix struct {
	board target.Board
}

var SmartLED SmartMatrix

func (s *SmartMatrix) Configure() error {

	if err := s.board.Configure(
		hub75.Matrix{
			Panel: []hub75.Panel{
				hub75.Row32Scan16,
			},
			Pins: hub75.Pins{
				// Signal pins
				OEN: machine.HUB75_OEN_PIN,
				LAT: machine.HUB75_LAT_PIN,
				CLK: machine.HUB75_CLK_PIN,
				// Data pins
				R0: machine.HUB75_R0_PIN,
				G0: machine.HUB75_G0_PIN,
				B0: machine.HUB75_B0_PIN,
				R1: machine.HUB75_R1_PIN,
				G1: machine.HUB75_G1_PIN,
				B1: machine.HUB75_B1_PIN,
				// Address pins
				A0: machine.HUB75_A0_PIN,
				A1: machine.HUB75_A1_PIN,
				A2: machine.HUB75_A2_PIN,
				A3: machine.HUB75_A3_PIN,
				A4: machine.HUB75_A4_PIN,
			}}); err != nil {
		return err
	}

	return nil
}
