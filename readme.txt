README!

This is a matlab simulation of an inverted swinging pendulum with and without an LQR controller. There is a tuner file that will auto-tune the Q and R values.

Nesessary add-ons:
- Matlab Global Optimization Toolbox
- Matlab Control System Toolbox


To autotune:
1. Set system constants for gravity, inital length, mass, and dampening
2. Run the tuner
3. Upon completion, copy formatted Q and R values
4. Paste values into the pendulum file where it says "PASTE TUNER VALUES HERE"
5. Ensure system parameters in the pendulum simulation match the ones in the tuner
6. Run the simulation

To tune the tuner: adjust the score function to give higher/lower weighting to whicchever parameter you want

Limitations:
- Mass seems to be broken, any value much above 0.1kg renders the controller ineffective
