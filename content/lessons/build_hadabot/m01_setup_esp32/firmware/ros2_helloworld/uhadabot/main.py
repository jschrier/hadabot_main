from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG, PIN_CONFIG
from machine import Pin, PWM, I2C, Timer, ADC
import time
import _thread
import logging


logger = logging.getLogger(__name__)

hadabot = None


###############################################################################
class Controller:

    ###########################################################################
    def __init__(self, ros):

        # Motor pins
        self.pwm_pin_left = PWM(Pin(PIN_CONFIG["left"]["motor"]["pwm"]))
        self.pwm_pin_right = PWM(Pin(PIN_CONFIG["right"]["motor"]["pwm"]))

        self.fr_pin_left = Pin(PIN_CONFIG["left"]["motor"]["fr"], Pin.OUT)
        self.fr_pin_right = Pin(PIN_CONFIG["right"]["motor"]["fr"], Pin.OUT)

        self.pwm_pin_left.duty(0)
        self.pwm_pin_right.duty(0)
        self.fr_pin_left.off()
        self.fr_pin_right.off()


    ###########################################################################
    def turn_wheel(self, wheel_power_f32, pwm_pin, fr_pin, prev_direction):
        factor = max(min(wheel_power_f32, 1.0), -1.0)

        # Reversing direction? Stop first!
        if (prev_direction * wheel_power_f32) < 0.0:
            self._send_motor_signal(0.0, pwm_pin, fr_pin)
            time.sleep_ms(10)

        # Send command
        self._send_motor_signal(factor, pwm_pin, fr_pin)

    ###########################################################################
    def _send_motor_signal(self, factor, pwm_pin, fr_pin):
        if factor >= 0:
            # FR pin lo to go forward
            fr_pin.off()
            pwm_pin.duty(int(1023 * factor))
        else:
            # FR pin hi and 'reverse' pwm to go backwards
            fr_pin.on()
            pwm_pin.duty(1023 - int(1023 * (-1*factor)))

    ###########################################################################
    def _send_motor_signal(self, factor, pwm_pin, fr_pin):
        if factor >= 0:
            # FR pin lo to go forward
            fr_pin.off()
            pwm_pin.duty(int(1023 * factor))
        else:
            # FR pin hi and 'reverse' pwm to go backwards
            fr_pin.on()
            pwm_pin.duty(1023 - int(1023 * (-1*factor)))

    ###########################################################################
    def right_wheel_cb(self, wheel_power):
        left_right = 1  # right
        spin_dir = 1 # forward
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_right, self.fr_pin_right,
            spin_dir)

    ###########################################################################
    def left_wheel_cb(self, wheel_power):
        left_right = 0  # left
        spin_dir = 1 # forward
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_left, self.fr_pin_left,
            spin_dir)

    ###########################################################################
    def run_once(self):
        cur_ms = time.ticks_ms()


###############################################################################
class Hadabot:

    ###########################################################################
    def __init__(self):
        self.ros = None
        self.spin_timer = None
        print("starting minimal hadabot init")

        try:
            # Start sensors first since it takes some time to power up
            p_sensor_display_power = Pin(
                PIN_CONFIG["power"]["sensors"], Pin.OUT)
            p_sensor_display_power.on()

            # Setup ROS
            self.hadabot_ip_address = CONFIG["network"]["hadabot_ip_address"]
            self.builtin_led = Pin(PIN_CONFIG["led"]["status"], Pin.OUT)
            if CONFIG["network"]["as_ap"] is True:
                import network
                ap = network.WLAN(network.AP_IF)
                while len(ap.status('stations')) == 0:
                    logger.info("Waiting for the machine running the " +
                                "ros2 web bridge to connect to the network...")
                    time.sleep(5)
            self.ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])

        
            # Publish out log info
            self.log_info = Topic(self.ros, "hadabot/log/info", "std_msgs/String")

            # Controller
            self.controller = Controller(self.ros)


            # Subscribe to blink led
            self.blink_led_sub_topic = Topic(
                self.ros, "hadabot/blink_led", "std_msgs/Int32")
            self.blink_led_sub_topic.subscribe(self.blink_led_cb)

            # Subscribe to wheel turn callbacks
            self.rw_sub_topic = Topic(
                self.ros, "hadabot/wheel_power_right", "std_msgs/Float32")
            self.rw_sub_topic.subscribe(self.controller.right_wheel_cb)
            self.lw_sub_topic = Topic(
                self.ros, "hadabot/wheel_power_left", "std_msgs/Float32")
            self.lw_sub_topic.subscribe(self.controller.left_wheel_cb)

            self.last_hb_ms = time.ticks_ms()
            self.last_controller_ms = time.ticks_ms()

            # Boot button to stop hadabot timer
            self.need_shutdown = False
            self.boot_button = Pin(PIN_CONFIG["button"]["boot"], Pin.IN)
            self.boot_button.irq(trigger=Pin.IRQ_RISING,
                                 handler=self.user_shutdown_cb)

            # Spin thread
            _thread.start_new_thread(self.spin, ())

        except Exception as ex:
            logger.error("Init exception hit {}".format(str(ex)))
            self.blink_led(5, end_off=True)
            self.shutdown()
            raise ex

    ###########################################################################
    def user_shutdown_cb(self, pin):
        logger.info("User shutdown invoked")
        self.need_shutdown = True

    ###########################################################################
    def ping_time_reference_cb(self, msg):
        self.ping_time_reference_ack_topic.publish(Message(msg))
        self.last_hb_ms = time.ticks_ms()

    ###########################################################################
    def ping_inertia_stamped_cb(self, msg):
        self.ping_inertia_stamped_ack_topic.publish(Message(msg))
        self.last_hb_ms = time.ticks_ms()

    ###########################################################################
    def blink_led_cb(self, msg):
        ntimes = int(msg["data"])
        self.blink_led(ntimes, end_off=False)

    def blink_led(self, ntimes, end_off):
        # Start off
        self.builtin_led.off()
        time.sleep(0.5)

        for x in range(0, ntimes):
            self.builtin_led.off()
            time.sleep(0.25)
            self.builtin_led.on()
            time.sleep(0.5)

        if end_off:
            self.builtin_led.off()

    ###########################################################################
    def shutdown(self):
        # Shutdown
        self.need_shutdown = True

        if self.ros is not None:
            try:
                self.ros.terminate()
            except Exception:
                pass
            self.ros = None

        self.builtin_led.off()

    ###########################################################################
    def spin(self):
        go = True
        while go:
            try:
                if self.need_shutdown:
                    raise Exception("Shut down requested")

                self.ros.run_once()

                cur_ms = time.ticks_ms()

                # Run the controller
                if time.ticks_diff(cur_ms, self.last_controller_ms) > 50:
                    self.controller.run_once()
                    self.last_controller_ms = cur_ms

                # Publish heartbeat message
                if time.ticks_diff(cur_ms, self.last_hb_ms) > 5000:
                    # logger.info("Encoder left - {}".format(en_count_left))
                    # logger.info("Encoder right - {}".format(en_count_right))
                    # self.log_info.publish(Message(
                    #    "Hadabot heartbeat - IP {}".format(
                    #        self.hadabot_ip_address)))
                    self.log_info.publish(Message(str(cur_ms)))
                    self.last_hb_ms = cur_ms

            except Exception as ex:
                logger.error("Spin exception hit {}".format(str(ex)))
                go = False

                # If this was not a user triggered shutdown then reset
                if self.need_shutdown is False:
                    logger.error(
                        "Soft resetting doesn't quite work so just shutdown")
                    # machine.soft_reset()
                else:
                    ex = None  # User requested shutdown

                self.shutdown()

                if ex is not None:
                    raise ex


###############################################################################
def main(argv):
    global hadabot
    hadabot = Hadabot()


main(None)
