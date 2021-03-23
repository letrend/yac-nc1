#include "joystick.hpp"

JoyStick::JoyStick(){
        joystick_device = "/dev/input/js0";
        js = open(joystick_device, O_RDONLY);

        if (js == -1)
                ROS_INFO("Could not open joystick");
}


int JoyStick::read_event(int fd, struct js_event *event)
{
        ssize_t bytes;

        bytes = read(fd, event, sizeof(*event));

        if (bytes == sizeof(*event))
                return 0;

        /* Error, could not read full event. */
        return -1;
}

size_t JoyStick::get_axis_count(int fd)
{
        __u8 axes;

        if (ioctl(fd, JSIOCGAXES, &axes) == -1)
                return 0;

        return axes;
}

size_t JoyStick::get_button_count(int fd)
{
        __u8 buttons;
        if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
                return 0;

        return buttons;
}

size_t JoyStick::get_axis_state(struct js_event *event, struct axis_state axes[3])
{
        size_t axis = event->number / 2;

        if (axis < 3)
        {
                if (event->number % 2 == 0)
                        axes[axis].x = event->value;
                else
                        axes[axis].y = event->value;
        }

        return axis;
}
