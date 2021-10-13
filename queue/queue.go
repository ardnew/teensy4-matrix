package queue

import "runtime/volatile"

type Ring struct {
	size  volatile.Register32
	start volatile.Register32
	count volatile.Register32
}

func (r *Ring) Init(size uint32) {
	r.size.Set(size)
	r.start.Set(0)
	r.count.Set(0)
}

func (r *Ring) Full() bool {
	return r.count.Get() == r.size.Get()
}

func (r *Ring) Empty() bool {
	return r.count.Get() == 0
}

func (r *Ring) NextRead() uint32 {
	return r.start.Get()
}

func (r *Ring) NextWrite() uint32 {
	return (r.start.Get() + r.count.Get()) % r.size.Get()
}

func (r *Ring) Read() {
	r.start.Set((r.start.Get() + 1) % r.size.Get())
	r.count.Set(r.count.Get() - 1)
}

func (r *Ring) Write() {
	if r.count.Get() == r.size.Get() {
		r.start.Set((r.start.Get() + 1) % r.size.Get())
	} else {
		r.count.Set(r.count.Get() + 1)
	}
}
