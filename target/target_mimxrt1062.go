// +build mimxrt1062

package target

import (
	"device/nxp"
	"machine"
)

var flexIOPin = map[*nxp.FlexIO]map[machine.Pin]uint8{
	&nxp.FlexIO1: {
		machine.PD0: 0, machine.PD1: 1, machine.PD2: 2, machine.PD3: 3,
		machine.PD4: 4, machine.PD5: 5, machine.PD6: 6, machine.PD7: 7,
		machine.PD8: 8, machine.PD9: 9, machine.PD10: 10, machine.PD11: 11,
		machine.PD26: 12, machine.PD27: 13, machine.PD28: 14, machine.PD29: 15,
	},
	&nxp.FlexIO2: {
		machine.PB0: 0, machine.PB1: 1, machine.PB2: 2, machine.PB3: 3,
		machine.PB4: 4, machine.PB5: 5, machine.PB6: 6, machine.PB7: 7,
		machine.PB8: 8, machine.PB9: 9, machine.PB10: 10, machine.PB11: 11,
		machine.PB12: 12, machine.PB13: 13, machine.PB14: 14, machine.PB15: 15,
		machine.PB16: 16, machine.PB17: 17, machine.PB18: 18, machine.PB19: 19,
		machine.PB20: 20, machine.PB21: 21, machine.PB22: 22, machine.PB23: 23,
		machine.PB24: 24, machine.PB25: 25, machine.PB26: 26, machine.PB27: 27,
		machine.PB28: 28, machine.PB29: 29, machine.PB30: 30, machine.PB31: 31,
	},
	&nxp.FlexIO3: {
		machine.PA16: 0, machine.PA17: 1, machine.PA18: 2, machine.PA19: 3,
		machine.PA20: 4, machine.PA21: 5, machine.PA22: 6, machine.PA23: 7,
		machine.PA24: 8, machine.PA25: 9, machine.PA26: 10, machine.PA27: 11,
		machine.PA28: 12, machine.PA29: 13, machine.PA30: 14, machine.PA31: 15,
		machine.PB16: 16, machine.PB17: 17, machine.PB18: 18, machine.PB19: 19,
		machine.PB20: 20, machine.PB21: 21, machine.PB22: 22, machine.PB23: 23,
		machine.PB24: 24, machine.PB25: 25, machine.PB26: 26, machine.PB27: 27,
		machine.PB28: 28, machine.PB29: 29, machine.PB30: 30, machine.PB31: 31,
	},
}

func FlexIOPin(pin machine.Pin) (*nxp.FlexIO, uint8) {
	for f, m := range flexIOPin {
		if p, ok := m[pin]; ok {
			return f, p
		}
	}
	return nil, uint8(machine.NoPin)
}
