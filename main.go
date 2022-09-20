package main

import (
	"math/rand"
	"time"

	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
)

type Boid struct {
	position     pixel.Vec
	velocity     pixel.Vec
	acceleration pixel.Vec
}

func NewBoid() *Boid {
	pos := pixel.V(rand.Float64()*768, rand.Float64()*1024)
	vel := pixel.V((rand.Float64()-0.5)*10, (rand.Float64()-0.5)*10)
	acc := pixel.V((rand.Float64()-0.5)/2, (rand.Float64()-0.5)/2)
	return &Boid{pos, vel, acc}
}

type Flock struct {
	boids []Boid
}

func NewFlock(size int) *Flock {
	boids := []Boid{}
	for i := 0; i < size; i++ {
		boid := NewBoid()
		boids = append(boids, *boid)
	}
	return &Flock{boids}
}

type FlockController struct {
	flock       Flock
	maxX        float64
	maxY        float64
	maxVelocity float64
	maxForce    float64
	perception  float64
}

func (controller *FlockController) begin() {
	flock := controller.flock
	for i := range flock.boids {
		go controller.applyBehavior(i)
		go controller.reposition(i)
		go controller.wrap(i)
	}
}

func (controller *FlockController) reposition(i int) {
	for {
		boid := &controller.flock.boids[i]

		boid.position = boid.position.Add(boid.velocity)
		boid.velocity = boid.velocity.Add(boid.acceleration)

		if boid.velocity.Len() > float64(controller.maxVelocity) {
			boid.velocity = boid.velocity.Map(func(v float64) float64 { return v / boid.velocity.Len() * controller.maxVelocity })
		}

		boid.acceleration = pixel.V(0, 0)

		time.Sleep(time.Second / 60)
	}
}

func (controller *FlockController) applyBehavior(i int) {
	for {
		controller.alignment(i)
		controller.cohesion(i)
		controller.separation(i)

		time.Sleep(time.Second / 60)
	}
}

func (controller *FlockController) alignment(i int) {
	boids := controller.flock.boids
	curBoid := &boids[i]
	steering := pixel.V(0, 0)
	total := 0
	avgVec := pixel.V(0, 0)
	for _, boid := range boids {
		if curBoid.position.Sub(boid.position).Len() < controller.perception {
			avgVec = avgVec.Add(boid.velocity)
			total++
		}
	}

	if total > 0 {
		avgVec = avgVec.Map(func(v float64) float64 { return v / float64(total) })
		avgVec = avgVec.Map(func(v float64) float64 { return v / avgVec.Len() * controller.maxVelocity })
		steering = avgVec.Sub(curBoid.velocity)
	}

	curBoid.acceleration = curBoid.acceleration.Add(steering)
}

func (controller *FlockController) cohesion(i int) {
	boids := controller.flock.boids
	curBoid := &boids[i]
	steering := pixel.V(0, 0)
	total := 0
	avgPos := pixel.V(0, 0)
	for _, boid := range boids {
		if boid.position.Sub(curBoid.position).Len() < controller.perception {
			avgPos = avgPos.Add(boid.position)
			total++
		}
	}

	if total > 0 {
		avgPos = avgPos.Map(func(v float64) float64 { return v / float64(total) })
		vecToAvgPos := avgPos.Sub(curBoid.position)
		if vecToAvgPos.Len() > 0 {
			vecToAvgPos = vecToAvgPos.Map(func(v float64) float64 { return v / vecToAvgPos.Len() * controller.maxVelocity })
		}
		steering = vecToAvgPos.Sub(curBoid.velocity)
		if steering.Len() > controller.maxForce {
			steering = steering.Map(func(v float64) float64 { return v / steering.Len() * controller.maxForce })
		}
	}

	curBoid.acceleration = curBoid.acceleration.Add(steering)
}

func (controller *FlockController) separation(i int) {
	boids := controller.flock.boids
	curBoid := &boids[i]
	steering := pixel.V(0, 0)
	total := 0
	avgVec := pixel.V(0, 0)
	for _, boid := range boids {
		distance := boid.position.Sub(curBoid.position).Len()
		if !boid.position.Eq(curBoid.position) && distance < controller.perception {
			diff := curBoid.position.Sub(boid.position)
			diff = diff.Map(func(v float64) float64 { return v / distance })
			avgVec = avgVec.Add(diff)
			total++
		}
	}

	if total > 0 {
		avgVec = avgVec.Map(func(v float64) float64 { return v / float64(total) })
		if avgVec.Len() > 0 {
			avgVec = avgVec.Map(func(v float64) float64 { return v / avgVec.Len() * controller.maxVelocity })
		}
		steering = avgVec.Sub(curBoid.velocity)
		if steering.Len() > controller.maxForce {
			steering = steering.Map(func(v float64) float64 { return v / steering.Len() * controller.maxForce })
		}
	}

	curBoid.acceleration = curBoid.acceleration.Add(steering)
}

func (controller *FlockController) wrap(i int) {
	for {
		boid := &controller.flock.boids[i]
		if boid.position.X > controller.maxX {
			boid.position.X = 0
		} else if boid.position.X < 0 {
			boid.position.X = controller.maxX
		}

		if boid.position.Y > controller.maxY {
			boid.position.Y = 0
		} else if boid.position.Y < 0 {
			boid.position.Y = controller.maxY
		}

		time.Sleep(time.Second / 60)
	}
}

func setup(width float64, height float64) *pixelgl.Window {
	cfg := pixelgl.WindowConfig{
		Title:  "Pixel Rocks!",
		Bounds: pixel.R(0, 0, width, height),
		VSync:  true,
	}
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}
	return win
}

func run() {
	width, height := 1024.0, 768.0
	win := setup(width, height)

	imd := imdraw.New(nil)

	flock := NewFlock(400)
	flockController := FlockController{*flock, width, height, 5, 1, 100}
	flockController.begin()

	for !win.Closed() {
		win.Clear(pixel.RGB(0, 0, 0))
		imd.Clear()

		for _, boid := range flock.boids {
			imd.Push(boid.position)
		}

		imd.Circle(1, 0)
		imd.Draw(win)
		win.Update()

	}
}

func main() {
	pixelgl.Run(run)
}
