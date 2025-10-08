from typing import override
import math

from rlbot.flat import BallAnchor, ControllerState, GamePacket, PlayerInfo
from rlbot.managers import Bot
from rlbot_flatbuffers import CarAnchor
from util.orientation import Orientation, relative_location

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target, angle_to_target
from util.sequence import ControlStep, Sequence
from util.vec import Vec3



class MyBot(Bot):
    active_sequence: Sequence | None = None
    boost_pad_tracker: BoostPadTracker = BoostPadTracker()

    @override
    def initialize(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.field_info)

    @override
    def get_output(self, packet: GamePacket) -> ControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        if len(packet.balls) == 0:
            # If there are no balls current in the game (likely due to being in a replay), skip this tick.
            return ControllerState()
        # we can now assume there's at least one ball in the match

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            return self.active_sequence.tick(packet)

        # Gather some information about our car and the ball
        my_car = packet.players[self.index]
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        ball_location = Vec3(packet.balls[0].physics.location)
        blueteam = my_car.team == 0
        redteam = my_car.team == 1

        if ball_location == Vec3(0, 0, 0):
            is_kickoff = True
        else:
            is_kickoff = False

        # By default we will chase the ball, but target_location can be changed later
        target_location = ball_location
        
        

        self.renderer.begin_rendering()

        if car_location.dist(ball_location) > 1500:
            # We're far away from the ball, let's try to lead it a little bit
            # self.ball_prediction can predict bounces, etc
            ball_in_future = find_slice_at_time(
                self.ball_prediction, packet.match_info.seconds_elapsed + 2
            )

            # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
            # replays, so check it to avoid errors.
            if ball_in_future is not None:
                target_location = Vec3(ball_in_future.physics.location)

                # BallAnchor(0) will dynamically start the point at the ball's current location
                # 0 makes it reference the ball at index 0 in the packet.balls list
                self.renderer.draw_line_3d(
                    BallAnchor(0), target_location, self.renderer.cyan
                )
        
        #sets the defensive side to the bot's side of the field
        if blueteam:
            defensivepos = Vec3(ball_location.x.__mul__(-0.25), -5120, 0)
            if car_location.y > ball_location.y:
                target_location = defensivepos
        if redteam:
            defensivepos = Vec3(ball_location.x.__mul__(-0.25), 5120, 0)
            if car_location.y < ball_location.y:
                target_location = defensivepos
        
        #if the car has no boost, go get some
        if my_car.boost == 0 and is_kickoff == False:
            nearest_pad = sorted(self.boost_pad_tracker.boost_pads, key=lambda pad: pad.location.dist(car_location))[0]
            if nearest_pad.is_active:
                target_location = nearest_pad.location
        
        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(
            CarAnchor(self.index), target_location, self.renderer.white
        )
        self.renderer.draw_string_3d(
            f"Speed: {car_velocity.length():.1f}",
            CarAnchor(self.index),
            1,
            self.renderer.white,
        )
        self.renderer.draw_line_3d(
            target_location - Vec3(0, 0, 50),
            target_location + Vec3(0, 0, 50),
            self.renderer.cyan,
        )

        self.renderer.end_rendering()

        """if 800 < car_velocity.length() < 850 and car_location.dist(ball_location) > 1000 and not is_kickoff:
            # We'll do a front flip if the car is moving at a certain speed.
            return self.begin_front_flip(packet)
        """
        controls = ControllerState()
        controls.steer = steer_toward_target(my_car, target_location)
        controls.throttle = 1.0

        #if the bot is facing the target, boost. if not, handbrake turn
        if abs(angle_to_target(my_car, target_location)) < 90:
            controls.boost = True
            controls.handbrake = False
        else:
            controls.boost = False
            controls.handbrake = True

        return controls

    def begin_front_flip(self, packet: GamePacket) -> ControllerState:
        # Send some quickchat just for fun
        # There won't be any content of the message for other bots,
        # but "I got it!" will be display for a human to see!
        # self.send_match_comm(b"", "I got it!")

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence(
            [
                ControlStep(duration=0.05, controls=ControllerState(jump=True)),
                ControlStep(duration=0.05, controls=ControllerState(jump=False)),
                ControlStep(
                    duration=0.2, controls=ControllerState(jump=True, pitch=-1)
                ),
                ControlStep(duration=0.8, controls=ControllerState()),
            ]
        )

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)


if __name__ == "__main__":
    # Connect to RLBot and run
    # Having the agent id here allows for easier development,
    # as otherwise the RLBOT_AGENT_ID environment variable must be set.
    MyBot("vanster6/KitBot").run()