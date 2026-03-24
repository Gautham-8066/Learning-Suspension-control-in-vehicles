# Learning-Suspension-control-in-vehicles
I designed a Closed-Loop Active Suspension Controller for a quarter-car model. I used State-Space representation to model vehicle dynamics and implemented an LQR algorithm to optimize the trade-off between passenger comfort and tire road-holding. I then verified the design using stochastic road profiles in MATLAB.

In a standard car, the suspension is Passive. It consists of a metal spring and a hydraulic shock absorber. When you hit a bump, the spring compresses to soak up the energy, and the damper stops the car from bouncing forever.

In our project, we model one-fourth of the car.

Sprung Mass: The heavy car body and passengers.

Unsprung Mass: The wheel and tire.

The Actuator (The Muscle): This is the "Active" part. It’s a hydraulic or electric motor placed parallel to the spring that can push or pull the car body instantly.

We describe the car's "state" at any microsecond using four variables: how much the suspension is stretched, how fast the body is moving, how much the tire is squished, and how fast the wheel is spinning.

The Linear Quadratic Regulator is an optimization algorithm. Imagine a scale. On one side is Passenger Comfort (keeping the body perfectly still). On the other is Road Holding (keeping the tire glued to the asphalt).


The plots show that the car body barely moved when hitting a massive bump. The cost is the wheel had to move much more violently. Because the body stayed still, the tire had to absorb the entire 8cm bump by itself.
<img width="1675" height="908" alt="image" src="https://github.com/user-attachments/assets/cedcc179-3db3-408d-9bfd-71eba63d0150" />
