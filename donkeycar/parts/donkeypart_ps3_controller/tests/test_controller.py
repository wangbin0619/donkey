
#import pytest
from donkeycar.parts.donkeypart_ps3_controller.donkeypart_ps3_controller.part import PS3Joystick, PS3JoystickController

def test_ps3_joystick():
    js = PS3Joystick()
    assert js is not None
    js.init()

def test_ps3_joystick_controller():
    js = PS3JoystickController()
    assert js is not None
    js.init_js()
    js.run_threaded(None)
    js.print_controls()
    def test_fn():
        pass
    js.set_button_down_trigger("x", test_fn)
    js.erase_last_N_records()
    js.on_throttle_changes()
    js.emergency_stop()
    #js.update()
    js.set_steering(0.0)
    js.set_throttle(0.0)
    js.toggle_manual_recording()
    js.increase_max_throttle()
    js.decrease_max_throttle()
    js.toggle_constant_throttle()
    js.toggle_mode()
    js.chaos_monkey_on_left()
    js.chaos_monkey_on_right()
    js.chaos_monkey_off()

if __name__ == '__main__':
    test_ps3_joystick()
    test_ps3_joystick_controller()