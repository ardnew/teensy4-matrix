package hub75

import (
	"machine"
)

type Matrix struct {
	Panel []Panel
	Pins  Pins
}

type Pins struct {
	// Signal pins
	OEN machine.Pin
	LAT machine.Pin
	CLK machine.Pin
	// Data pins
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

type Panel uint8

const (
	Row32Scan16 Panel = iota
	Row16Scan8
	Row64Scan32
	Row4Scan2
	Row8Scan4
	Row16Col32Scan2
	_
	Row16Col32Scan4
	Row2Scan1
	Row16Col32Scan4v2
	Row32Col64Scan8
	Row64Col64Scan16
	Row16Col32Scan4v3
	Row16Col32Scan4v4
)

const (
	DefaultPanelWidth = 32

	RefreshDepth          = 24
	ParallelColorChannels = 2
	ColorChannelsPerPixel = 3 // red, green, and blue

	// LatchesPerRow computes the color depth per pixel for a given refresh depth.
	// For example, LatchesPerRow=8 when RefreshDepth=24, which corresponds to
	// 24-bit TrueColor (8 bits per pixel).
	LatchesPerRow = RefreshDepth / ColorChannelsPerPixel
)

// MinRefreshRate returns the minimum refresh rate of the receiver panel for a
// given timer frequency and refresh depth.
// The HUB75 panel cannot refresh slower than this due to PWM register overflow.
func (p Panel) MinRefreshRate(timerFreq uint32) uint32 {
	lat := uint32(1) << LatchesPerRow
	return ((timerFreq / 0xFFFF * lat / (lat - 1) / (p.ScanMod()) / 2) + 1)
}

func (p Panel) MultiRowRefresh() bool {
	return p.PhysicalRowsPerRefresh() > 1
}

func (p Panel) PhysicalRowsPerRefresh() uint32 {
	return p.Height() / p.ScanMod() / ParallelColorChannels
}

func (p Panel) PixelsPerLatch() uint32 {
	return p.PhysicalRowsPerRefresh() * p.Width()
}

func (p Panel) Size() (width, height uint32) {
	return p.Width(), p.Height()
}

func (p Panel) Width() uint32 {
	switch p {
	case Row32Scan16:
		return DefaultPanelWidth
	case Row64Col64Scan16:
		return 64
	case Row16Scan8:
		return DefaultPanelWidth
	case Row16Col32Scan2:
		return 32
	case Row4Scan2:
		return DefaultPanelWidth
	case Row8Scan4:
		return DefaultPanelWidth
	case Row32Col64Scan8:
		return 64
	case Row16Col32Scan4:
		return 32
	case Row16Col32Scan4v2:
		return 32
	case Row16Col32Scan4v3:
		return 32
	case Row16Col32Scan4v4:
		return 32
	case Row2Scan1:
		return DefaultPanelWidth
	case Row64Scan32:
		return DefaultPanelWidth
	}
	return 0
}

func (p Panel) Height() uint32 {
	switch p {
	case Row32Scan16:
		return 32
	case Row64Col64Scan16:
		return 64
	case Row16Scan8:
		return 16
	case Row16Col32Scan2:
		return 16
	case Row4Scan2:
		return 4
	case Row8Scan4:
		return 8
	case Row32Col64Scan8:
		return 32
	case Row16Col32Scan4:
		return 16
	case Row16Col32Scan4v2:
		return 16
	case Row16Col32Scan4v3:
		return 16
	case Row16Col32Scan4v4:
		return 16
	case Row2Scan1:
		return 2
	case Row64Scan32:
		return 64
	}
	return 0
}

func (p Panel) ScanMod() uint32 {
	switch p {
	case Row32Scan16:
		return 16
	case Row64Col64Scan16:
		return 16
	case Row16Scan8:
		return 8
	case Row16Col32Scan2:
		return 2
	case Row4Scan2:
		return 2
	case Row8Scan4:
		return 4
	case Row16Col32Scan4:
		return 4
	case Row16Col32Scan4v2:
		return 4
	case Row16Col32Scan4v3:
		return 4
	case Row16Col32Scan4v4:
		return 4
	case Row32Col64Scan8:
		return 8
	case Row2Scan1:
		return 1
	case Row64Scan32:
		return 32
	}
	return 0
}

func (p Panel) RowPairOffset() uint32 {
	switch p {
	case Row32Scan16:
		return 16
	case Row64Col64Scan16:
		return 32
	case Row16Scan8:
		return 8
	case Row16Col32Scan2:
		return 8
	case Row4Scan2:
		return 2
	case Row8Scan4:
		return 4
	case Row32Col64Scan8:
		return 16
	case Row16Col32Scan4:
		return 8
	case Row16Col32Scan4v2:
		return 8
	case Row16Col32Scan4v3:
		return 8
	case Row16Col32Scan4v4:
		return 8
	case Row2Scan1:
		return 1
	case Row64Scan32:
		return 32
	}
	return 0
}
