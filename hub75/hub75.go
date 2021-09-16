package hub75

import (
	"machine"
)

type Pins struct {
	// Signal pins
	OEN machine.Pin
	LAT machine.Pin
	CLK machine.Pin
	// Color pins
	R0 machine.Pin
	G0 machine.Pin
	B0 machine.Pin
	R1 machine.Pin
	G1 machine.Pin
	B1 machine.Pin
	// Address pins
	A0 machine.Pin
	A1 machine.Pin
	A2 machine.Pin
	A3 machine.Pin
	A4 machine.Pin
}
