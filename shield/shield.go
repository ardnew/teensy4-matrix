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
				OEN: machine.D2,
				LAT: machine.D3,
				CLK: machine.D7,
				// Data pins
				R0: machine.D6,
				G0: machine.D9,
				B0: machine.D10,
				R1: machine.D12,
				G1: machine.D11,
				B1: machine.D13,
				// Address pins
				A0: machine.D6,
				A1: machine.D9,
				A2: machine.D10,
				A3: machine.D12,
				A4: machine.D11,
			}}); err != nil {
		return err
	}

	return nil
}
