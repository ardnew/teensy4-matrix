package hub75

import (
	"machine"
)

type Pins struct {
	OE  machine.Pin
	LAT machine.Pin
	CLK machine.Pin
	R0  machine.Pin
	G0  machine.Pin
	B0  machine.Pin
	R1  machine.Pin
	G1  machine.Pin
	B1  machine.Pin
}
