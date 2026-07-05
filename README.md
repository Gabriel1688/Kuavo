![image](https://github.com/user-attachments/assets/5a1df558-a776-4f96-9bef-4465948a6ecd)

Although the physics engine is important, modeling robots requires much more than just realistic and performant physics.
Drake also provides a library of sensor models, actuator models, low-level controllers, and low-level perception
algorithms. For example, we have models for relatively simple sensors, like a rotary encoder or inertial measurement
unit. But we also have complex sensor models like simulated cameras which require a full rendering pipeline. We even
support a variety of camera models, including high-speed lightweight shader-based renderers that are ideal for many
simulation workflows, and full physics-based renderers that can be used for training and testing perception algorithms.

Building on the far-reaching success of model-based design frameworks like Simulink and Modelica, we encapsulate each of
these models as a “system” that can be easily composed into more complicated systems in a block diagram. To be a fully
compliant element in the Drake systems framework, each model must declare all of its state variables and pull any
randomness through an input port.

Compared to authoring a ROS node, this asks a little more of the system author up front. But the downstream benefits are
immense. The extra structure enables fully deterministic replay and more advanced design and analysis techniques. Once
you have a system, whether it’s a simple system or a complex diagram, you can easily use that system in a multiprocess
network passing framework (like ROS), or run it all in a single process for deterministic execution and debugging.
Drake’s collection of carefully vetted models is growing continuously, and we welcome your contributions!

The systems framework provides the abstraction and encapsulation that allows a mature software project to scale. There
are numerous robotics companies, from startups to large industry players, that are using Drake now in production. They
often tell me that they started using Drake because of the physics engine, but that it’s the systems framework that has
really enabled them to grow and scale.

The systems framework provides a software engineering abstraction, but it also provides a powerful mathematical
abstraction that enables more advanced algorithms. When I think of a simple, discrete-time dynamical system in
state-space form, I think of equations of the form:
x[n+1] = f(n, x[n], u[n], p, w[n]),
y[n] = g(n, x[n], u[n], p, w[n]),
where x is the state, u is an input, y are the outputs, p are the parameters, and w represents (potentially random)
disturbance inputs. Drake supports much more complex systems than this, with mixtures of continuous and discrete
dynamics, event handling, and abstract (structured) state types. But for systems that do admit this form, including the
composition of a time-stepping model of multibody dynamics with numerous sensors, and even a feedback controller, Drake
goes to some lengths to make the simple structured form of the equations available.

Given a clean mathematical model, it becomes clear how we formulate solutions to some fairly sophisticated questions
about the model:

Simulation is solving for x[⋅] given x[0], u[⋅], and p.
Planning or trajectory optimization is searching for {u[⋅],x[⋅]} (with w[⋅]=0). Robust planning includes taking an
expectation or worst case over w.
State estimation is searching for {x[⋅],w[⋅]}
System identification is searching for {p, w[⋅], and often x[⋅]}
Stability analysis is (for w=0), for instance, finding a set of initial conditions x[0] for which lim n→ ∞, x[n] → 0.
Stochastic stability analysis asks, e.g. the probability of leaving a region over some finite-time horizon.
Verification / falsification asks if there exists a w[⋅] such that ∃n s.t. x[n] ∈ failure set.
Some people would say that the problems we are tackling now in robotics are too complex to be treated with clean
mathematics. I strongly disagree. I think it’s precisely because the systems are so complex that we must think more
clearly about our formulations. Drake was designed to help bridge the gap between the clean mathematics and the
incredibly complex problems in robotics.

Optimization Framework
The last major component of Drake is the optimization framework, which provides a front-end to easily write mathematical
programs and then dispatch them to open-source or commercial solvers. In this sense, the optimization framework plays a
role like CVX or Yalmip in MATLAB, and JuMP in Julia. We support a hierarchy of convex optimization problems,
mixed-integer optimization, and general nonlinear optimization.

Formulating a simple mathematical program in Drake. We also make it easy to add costs and constraints from the physics
engine, and to solve common optimization problems from control.
The systems framework and the optimization framework share a common core of templatized scalar types to support
automatic differentiation and symbolic computation. Perhaps the best way to illustrate the power of this is through an
example: the direct trajectory optimization code. We can easily set up a trajectory optimization problem by handing it
as a system (which can be an entire diagram), and then adding costs and constraints. If the system happens to have
linear state dynamics, and the costs and constraints are all convex, then the trajectory optimizer will automatically
dispatch the problem to a high-performance convex optimization library. If not, the optimizer will dispatch to a more
generic nonlinear solver automatically.

Some people today believe that stochastic gradient descent (and its variants) are the only algorithms required in
robotics. Drake is wired to provide analytical gradients, even in complex systems. I personally believe there are also
rich untapped connections between mechanical systems (both smooth and non-smooth) and the fields of combinatorial
optimization and algebraic geometry. Drake is a great playground for exploring these ideas, and applying them to complex
systems.

Algorithms for perception, planning, and control
These three core components fit together to enable cutting edge research in advanced algorithms for robotics. For some
of the more mature algorithms, we provide implementations in Drake. Some examples include: solutions to Lyapunov and
Riccati equations for linear systems, various transcriptions of trajectory optimization, value iteration, and
sums-of-squares optimization for reachability and region of attraction analysis.

But not all of these can nor should live in Drake. Some of them have library requirements that would be too heavy to
include directly in Drake. For instance, we don’t demand PyTorch nor ROS as a dependency, but provide examples of how to
use them together. A great example of this is TRI’s work on [Large Behavior Models] which builds a powerful imitation
learning framework in PyTorch enabled by a very solid robot control foundation built in Drake. We are now starting to
build the ecosystem of shared tools and repositories using Drake as a library. We have a number of tutorials on the
Drake website and are writing more. I teach two advanced robotics classes at MIT, one called Underactuated Robotics and
the other called Robotic Manipulation; both have extensive course notes with Jupyter notebook examples running Drake on
Google Colab

https://medium.com/toyotaresearch/drake-model-based-design-in-the-age-of-robotics-and-machine-learning-59938c985515
