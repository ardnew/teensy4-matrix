package main

import (
	"time"

	"github.com/ardnew/teensy40-matrix/shield"
)

func main() {
	if err := shield.SmartLED.Configure(); err != nil {
		halt(err)
	}
}

func halt(err error) {
	for {
		println(err.Error())
		time.Sleep(time.Second)
	}
}
