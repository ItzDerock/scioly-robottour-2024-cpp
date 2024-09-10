# Science Olympiad - Robot Tour

C++ code for the Science Olympiad Robot Tour Divison C event. Targeted at the RP2040 microcontroller using the pico-sdk library.

## Read the [blog post](https://derock.blog/post/science-olympiad-robot-tour?ref=github) to learn more about the hardware and software. Or check out this [demo video](https://youtu.be/HW2_rhXF9W8)
<a href="https://derock.blog/post/science-olympiad-robot-tour?ref=github">
  <img src="https://derock.blog/thumbs/scioly-robot-tour-2024.webp" alt="thumbnail" />
</a>

---

The plan is the following:
- User inputs a general path to follow
- Robot interpolates missing points to create a pure-pursuit path
- On button click, robot resets odometry position and follows pure-pursuit path.
  - During run, robot will constantly recalculate the required target velocity to match the requested time goal.

# Archetecture
The RP2040 has 2 cores and 2 PIOs. For simplicity, this program will not use an RTOS system since it should be easy to incorperate the required parallel tasks using the two cores of the RP2040.

## PIOs
The PIOs are responsible for tracking the encoder counts of each left/right wheel to reduce the amount of interrupts and context switches the main core needs to do.

## Main Core
Responsible for following the pure-pursuit path.

## Second Core
Will handle the odometry tracking.
