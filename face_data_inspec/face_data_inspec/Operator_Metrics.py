#!/usr/bin/env python3

"""
ROS 2 Operator Metrics Node

Inputs
------
/face_expressions
    std_msgs/msg/Float64MultiArray

    72 FaceExpression2 values in the order supplied by the user.

/eye_position
    std_msgs/msg/Int32MultiArray

    [left_x, left_y, right_x, right_y]

Outputs
-------
/operator_metrics
    std_msgs/msg/Float64MultiArray

    24 calculated operator-state features.

Purpose
-------
This node calculates low-latency behavioral features that can later
be used to estimate:

    - attentiveness
    - mental workload
    - tiredness/fatigue

The node does NOT directly claim that any individual metric is
"workload" or "attention". Instead, it produces features that can
be fed into a later trained/calibrated model.

No pupil diameter is required.

Input rate
----------
Approximately 30 Hz.
"""


import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray


class OperatorMetricsNode(Node):

    # ============================================================
    # FaceExpression2 indices
    #
    # These indices correspond exactly to the list supplied by the
    # user.
    # ============================================================

    FACE_INDEX = {
        "Invalid": 0,

        "BrowLowererL": 1,
        "BrowLowererR": 2,

        "CheekPuffL": 3,
        "CheekPuffR": 4,

        "CheekRaiserL": 5,
        "CheekRaiserR": 6,

        "CheekSuckL": 7,
        "CheekSuckR": 8,

        "ChinRaiserB": 9,
        "ChinRaiserT": 10,

        "DimplerL": 11,
        "DimplerR": 12,

        "EyesClosedL": 13,
        "EyesClosedR": 14,

        "EyesLookDownL": 15,
        "EyesLookDownR": 16,

        "EyesLookLeftL": 17,
        "EyesLookLeftR": 18,

        "EyesLookRightL": 19,
        "EyesLookRightR": 20,

        "EyesLookUpL": 21,
        "EyesLookUpR": 22,

        "InnerBrowRaiserL": 23,
        "InnerBrowRaiserR": 24,

        "JawDrop": 25,

        "JawSidewaysLeft": 26,
        "JawSidewaysRight": 27,

        "JawThrust": 28,

        "LidTightenerL": 29,
        "LidTightenerR": 30,

        "LipCornerDepressorL": 31,
        "LipCornerDepressorR": 32,

        "LipCornerPullerL": 33,
        "LipCornerPullerR": 34,

        "LipFunnelerLB": 35,
        "LipFunnelerLT": 36,

        "LipFunnelerRB": 37,
        "LipFunnelerRT": 38,

        "LipPressorL": 39,
        "LipPressorR": 40,

        "LipPuckerL": 41,
        "LipPuckerR": 42,

        "LipStretcherL": 43,
        "LipStretcherR": 44,

        "LipSuckLB": 45,
        "LipSuckLT": 46,

        "LipSuckRB": 47,
        "LipSuckRT": 48,

        "LipTightenerL": 49,
        "LipTightenerR": 50,

        "LipsToward": 51,

        "LowerLipDepressorL": 52,
        "LowerLipDepressorR": 53,

        "MouthLeft": 54,
        "MouthRight": 55,

        "NoseWrinklerL": 56,
        "NoseWrinklerR": 57,

        "OuterBrowRaiserL": 58,
        "OuterBrowRaiserR": 59,

        "UpperLidRaiserL": 60,
        "UpperLidRaiserR": 61,

        "UpperLipRaiserL": 62,
        "UpperLipRaiserR": 63,

        "TongueTipInterdental": 64,
        "TongueTipAlveolar": 65,

        "TongueFrontDorsalPalate": 66,
        "TongueMidDorsalPalate": 67,
        "TongueBackDorsalVelar": 68,

        "TongueOut": 69,
        "TongueRetreat": 70,

        "Max": 71,
    }

    # ============================================================
    # Constructor
    # ============================================================

    def __init__(self):

        super().__init__("operator_metrics_node")

        # ========================================================
        # ROS subscriptions
        # ========================================================

        # Facial expression input.
        self.face_subscription = self.create_subscription(
            Float64MultiArray,
            "/full_facial_input",
            self.face_callback,
            10
        )

        # Eye position input.
        #
        # Expected:
        #
        # [left_x, left_y, right_x, right_y]
        #
        self.gaze_subscription = self.create_subscription(
            Int32MultiArray,
            "/eye_pixels",
            self.gaze_callback,
            10
        )

        # ========================================================
        # ROS publisher
        # ========================================================

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/operator_metrics",
            10
        )

        # ========================================================
        # Configuration
        # ========================================================

        # Expected input rate.
        self.sample_rate = 30.0

        # --------------------------------------------------------
        # Face history
        #
        # 60 seconds is useful for PERCLOS/fatigue metrics.
        # --------------------------------------------------------

        self.face_window_seconds = 60.0

        self.face_window_size = int(
            self.sample_rate *
            self.face_window_seconds
        )

        # --------------------------------------------------------
        # Gaze history
        #
        # 30 seconds gives enough information for fixation,
        # saccade, dispersion and entropy measurements.
        # --------------------------------------------------------

        self.gaze_window_seconds = 30.0

        self.gaze_window_size = int(
            self.sample_rate *
            self.gaze_window_seconds
        )

        # --------------------------------------------------------
        # Eye closure thresholds
        #
        # FaceExpression2 values are assumed to be approximately
        # normalized from 0 to 1.
        #
        # These should be calibrated for the particular headset /
        # user if necessary.
        # --------------------------------------------------------

        self.eye_closed_threshold = 0.70
        self.eye_open_threshold = 0.30

        # --------------------------------------------------------
        # Fixation velocity threshold
        #
        # IMPORTANT:
        #
        # This value depends on the coordinate system used by your
        # eye tracker.
        #
        # 100 is only a starting point if the coordinates are
        # approximately screen pixels.
        #
        # This should be calibrated experimentally.
        # --------------------------------------------------------

        self.fixation_velocity_threshold = 100.0

        # Minimum time required for a fixation.
        self.minimum_fixation_duration = 0.100

        # --------------------------------------------------------
        # Gaze entropy grid
        #
        # The recent gaze area is divided into a 10 x 10 grid.
        # --------------------------------------------------------

        self.grid_x = 10
        self.grid_y = 10

        # ========================================================
        # Face state/history
        # ========================================================

        self.latest_face = None

        # Eye closure values over the last 60 seconds.
        self.eye_closure_history = deque(
            maxlen=self.face_window_size
        )

        # Jaw values over the last 60 seconds.
        self.jaw_history = deque(
            maxlen=self.face_window_size
        )

        # Duration of detected blinks.
        self.blink_durations = deque(
            maxlen=200
        )

        # Timestamp of each detected blink.
        self.blink_times = deque(
            maxlen=200
        )

        # Current blink state.
        self.blink_active = False

        # Time at which current blink started.
        self.blink_start_time = None

        # ========================================================
        # Gaze state/history
        # ========================================================

        self.latest_gaze = None

        # Gaze positions:
        #
        # (x, y, timestamp)
        #
        self.positions = deque(
            maxlen=self.gaze_window_size
        )

        # Gaze velocity history.
        self.velocities = deque(
            maxlen=self.gaze_window_size
        )

        # Distance between left and right eye gaze positions.
        self.vergence_history = deque(
            maxlen=self.gaze_window_size
        )

        # --------------------------------------------------------
        # Fixations
        # --------------------------------------------------------

        self.fixation_durations = deque(
            maxlen=200
        )

        self.fixation_amplitudes = deque(
            maxlen=200
        )

        self.fixation_active = False

        self.fixation_start_time = None

        self.fixation_start_position = None

        # --------------------------------------------------------
        # Saccades
        # --------------------------------------------------------

        self.saccade_durations = deque(
            maxlen=200
        )

        self.saccade_amplitudes = deque(
            maxlen=200
        )

        # --------------------------------------------------------
        # Previous gaze sample
        # --------------------------------------------------------

        self.last_x = None
        self.last_y = None
        self.last_gaze_time = None

        # ========================================================
        # Timer
        #
        # Publish the combined feature vector at approximately
        # 30 Hz.
        # ========================================================

        self.timer = self.create_timer(
            1.0 / self.sample_rate,
            self.publish_metrics
        )

        self.get_logger().info(
            "Operator Metrics Node started."
        )

        self.get_logger().info(
            "Face input: /face_expressions"
        )

        self.get_logger().info(
            "Eye input: /eye_position [lx, ly, rx, ry]"
        )

        self.get_logger().info(
            "Output: /operator_metrics"
        )

    # ============================================================
    # FACE PROCESSING
    # ============================================================

    def face_callback(self, msg):
        """
        Receive the 72 facial expression values.

        Facial expressions are stored and the eye closure/blink
        state is updated immediately.
        """

        # --------------------------------------------------------
        # Validate message
        # --------------------------------------------------------

        if len(msg.data) < 72:

            self.get_logger().warning(
                "Received fewer than 72 face expression values: "
                + str(len(msg.data))
            )

            return

        # Store the latest face data.
        self.latest_face = list(msg.data)

        # Current ROS time in seconds.
        now = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # --------------------------------------------------------
        # Eye closure
        # --------------------------------------------------------

        left_eye = self.face_value(
            "EyesClosedL"
        )

        right_eye = self.face_value(
            "EyesClosedR"
        )

        # Use both eyes to create a bilateral eye closure value.
        eye_closure = (
            left_eye +
            right_eye
        ) / 2.0

        self.eye_closure_history.append(
            eye_closure
        )

        # --------------------------------------------------------
        # Blink detection
        # --------------------------------------------------------

        if not self.blink_active:

            # Start a blink when both eyes are sufficiently
            # closed.
            if eye_closure >= self.eye_closed_threshold:

                self.blink_active = True

                self.blink_start_time = now

        else:

            # Blink ends when the eyes reopen.
            if eye_closure <= self.eye_open_threshold:

                self.blink_active = False

                if self.blink_start_time is not None:

                    duration = (
                        now -
                        self.blink_start_time
                    )

                    # Ignore extremely short false detections and
                    # extremely long closures.
                    if 0.05 <= duration <= 2.0:

                        self.blink_durations.append(
                            duration
                        )

                        self.blink_times.append(
                            now
                        )

                self.blink_start_time = None

    # ============================================================
    # Get facial expression value
    # ============================================================

    def face_value(self, name):
        """
        Return one facial expression value by name.
        """

        if self.latest_face is None:
            return 0.0

        index = self.FACE_INDEX[name]

        if index >= len(self.latest_face):
            return 0.0

        return float(
            self.latest_face[index]
        )

    # ============================================================
    # GAZE PROCESSING
    # ============================================================

    def gaze_callback(self, msg):
        """
        Receive both eye positions.

        Expected input:

            [left_x, left_y, right_x, right_y]

        A midpoint between the two eyes is calculated and used as
        the primary gaze position.

        The distance between the two eyes is also retained as a
        vergence-related feature.
        """

        # --------------------------------------------------------
        # Validate message
        # --------------------------------------------------------

        if len(msg.data) < 4:

            self.get_logger().warning(
                "Eye position must contain "
                "[left_x, left_y, right_x, right_y]"
            )

            return

        # --------------------------------------------------------
        # Extract both eyes
        # --------------------------------------------------------

        left_x = float(
            msg.data[0]
        )

        left_y = float(
            msg.data[1]
        )

        right_x = float(
            msg.data[2]
        )

        right_y = float(
            msg.data[3]
        )

        # --------------------------------------------------------
        # Calculate binocular/cyclopean gaze
        #
        # This is the midpoint between the two eyes.
        # --------------------------------------------------------

        x = (
            left_x +
            right_x
        ) / 2.0

        y = (
            left_y +
            right_y
        ) / 2.0

        # Save the latest complete gaze measurement.
        self.latest_gaze = (
            left_x,
            left_y,
            right_x,
            right_y,
            x,
            y
        )

        # Current timestamp.
        now = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # --------------------------------------------------------
        # Binocular separation
        #
        # This is useful as a relative vergence feature if both
        # eyes are represented in the same coordinate system.
        # --------------------------------------------------------

        vergence = math.sqrt(
            (left_x - right_x) ** 2 +
            (left_y - right_y) ** 2
        )

        self.vergence_history.append(
            vergence
        )

        # --------------------------------------------------------
        # First gaze sample
        # --------------------------------------------------------

        if self.last_gaze_time is None:

            self.last_x = x
            self.last_y = y
            self.last_gaze_time = now

            self.positions.append(
                (x, y, now)
            )

            return

        # Time since previous gaze sample.
        dt = (
            now -
            self.last_gaze_time
        )

        if dt <= 0.0:
            return

        # --------------------------------------------------------
        # Gaze displacement
        # --------------------------------------------------------

        displacement = math.sqrt(
            (x - self.last_x) ** 2 +
            (y - self.last_y) ** 2
        )

        # --------------------------------------------------------
        # Gaze velocity
        # --------------------------------------------------------

        velocity = (
            displacement /
            dt
        )

        self.velocities.append(
            velocity
        )

        # Store position history.
        self.positions.append(
            (x, y, now)
        )

        # --------------------------------------------------------
        # Determine whether the current sample is likely part of
        # a fixation.
        # --------------------------------------------------------

        is_fixating = (
            velocity <=
            self.fixation_velocity_threshold
        )

        if is_fixating:

            # A new fixation has started.
            if not self.fixation_active:

                self.fixation_active = True

                self.fixation_start_time = now

                self.fixation_start_position = (
                    x,
                    y
                )

        else:

            # A fixation has ended.
            if self.fixation_active:

                duration = (
                    now -
                    self.fixation_start_time
                )

                # Only count sufficiently long fixations.
                if (
                    duration >=
                    self.minimum_fixation_duration
                ):

                    amplitude = math.sqrt(
                        (
                            x -
                            self.fixation_start_position[0]
                        ) ** 2
                        +
                        (
                            y -
                            self.fixation_start_position[1]
                        ) ** 2
                    )

                    self.fixation_durations.append(
                        duration
                    )

                    self.fixation_amplitudes.append(
                        amplitude
                    )

                self.fixation_active = False

                # ------------------------------------------------
                # The transition from fixation to high velocity
                # movement is treated as a saccade.
                # ------------------------------------------------

                self.saccade_durations.append(
                    dt
                )

                self.saccade_amplitudes.append(
                    displacement
                )

        # --------------------------------------------------------
        # Save previous gaze position.
        # --------------------------------------------------------

        self.last_x = x
        self.last_y = y
        self.last_gaze_time = now

    # ============================================================
    # GAZE ENTROPY
    # ============================================================

    def calculate_gaze_entropy(self):
        """
        Calculate spatial gaze entropy.

        The current 30-second gaze history is divided into a
        10 x 10 grid.

        Higher entropy means gaze is distributed across more
        spatial regions.

        IMPORTANT:
        This is a behavioral feature, not a direct measurement
        of attention.
        """

        if len(self.positions) < 10:
            return 0.0

        # Get recent X/Y coordinates.
        xs = [
            position[0]
            for position in self.positions
        ]

        ys = [
            position[1]
            for position in self.positions
        ]

        min_x = min(xs)
        max_x = max(xs)

        min_y = min(ys)
        max_y = max(ys)

        range_x = (
            max_x -
            min_x
        )

        range_y = (
            max_y -
            min_y
        )

        # If the gaze barely moved, entropy is zero.
        if range_x <= 1.0 or range_y <= 1.0:
            return 0.0

        bins = {}

        # Assign each gaze point to a spatial bin.
        for x, y, _ in self.positions:

            bin_x = int(
                (
                    (x - min_x) /
                    range_x
                ) *
                self.grid_x
            )

            bin_y = int(
                (
                    (y - min_y) /
                    range_y
                ) *
                self.grid_y
            )

            # Prevent index 10 when a value is exactly at max.
            bin_x = max(
                0,
                min(
                    self.grid_x - 1,
                    bin_x
                )
            )

            bin_y = max(
                0,
                min(
                    self.grid_y - 1,
                    bin_y
                )
            )

            key = (
                bin_x,
                bin_y
            )

            bins[key] = (
                bins.get(key, 0) +
                1
            )

        # Calculate Shannon entropy.
        total = len(
            self.positions
        )

        entropy = 0.0

        for count in bins.values():

            probability = (
                count /
                total
            )

            if probability > 0.0:

                entropy -= (
                    probability *
                    math.log2(probability)
                )

        return entropy

    # ============================================================
    # PUBLISH METRICS
    # ============================================================

    def publish_metrics(self):
        """
        Calculate and publish the combined feature vector.

        The latest face and gaze measurements are used.

        The node publishes at approximately 30 Hz.
        """

        # --------------------------------------------------------
        # Do not publish until both inputs have been received.
        # --------------------------------------------------------

        if (
            self.latest_face is None or
            self.latest_gaze is None
        ):

            return

        now = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # ========================================================
        # FACE METRICS
        # ========================================================

        # --------------------------------------------------------
        # Bilateral eye closure
        # --------------------------------------------------------

        left_eye = self.face_value(
            "EyesClosedL"
        )

        right_eye = self.face_value(
            "EyesClosedR"
        )

        eye_closure = (
            left_eye +
            right_eye
        ) / 2.0

        # --------------------------------------------------------
        # PERCLOS
        #
        # Percentage/proportion of samples where the eyes are
        # considered closed.
        #
        # Current window = 60 seconds.
        # --------------------------------------------------------

        if self.eye_closure_history:

            closed_samples = sum(
                value >=
                self.eye_closed_threshold
                for value in
                self.eye_closure_history
            )

            perclos = (
                closed_samples /
                len(
                    self.eye_closure_history
                )
            )

        else:

            perclos = 0.0

        # --------------------------------------------------------
        # Blink rate
        #
        # Number of blinks in the most recent 60 seconds.
        # --------------------------------------------------------

        while (
            self.blink_times and
            now -
            self.blink_times[0]
            > 60.0
        ):

            self.blink_times.popleft()

        blink_rate = len(
            self.blink_times
        )

        # --------------------------------------------------------
        # Mean blink duration
        # --------------------------------------------------------

        if self.blink_durations:

            mean_blink_duration = (
                sum(
                    self.blink_durations
                ) /
                len(
                    self.blink_durations
                )
            )

        else:

            mean_blink_duration = 0.0

        # --------------------------------------------------------
        # Jaw drop
        # --------------------------------------------------------

        jaw_drop = self.face_value(
            "JawDrop"
        )

        # --------------------------------------------------------
        # Facial activity
        #
        # Mean absolute activation of the facial expressions.
        # --------------------------------------------------------

        facial_values = [
            abs(float(value))
            for value
            in self.latest_face[1:71]
        ]

        if facial_values:

            facial_activity = (
                sum(facial_values) /
                len(facial_values)
            )

        else:

            facial_activity = 0.0

        # --------------------------------------------------------
        # Facial asymmetry
        #
        # Average absolute difference between left/right
        # expression pairs.
        # --------------------------------------------------------

        paired_features = [

            ("BrowLowererL", "BrowLowererR"),

            ("CheekPuffL", "CheekPuffR"),

            ("CheekRaiserL", "CheekRaiserR"),

            ("CheekSuckL", "CheekSuckR"),

            ("DimplerL", "DimplerR"),

            ("EyesClosedL", "EyesClosedR"),

            ("LidTightenerL", "LidTightenerR"),

            ("LipCornerDepressorL",
             "LipCornerDepressorR"),

            ("LipCornerPullerL",
             "LipCornerPullerR"),

            ("LipPressorL",
             "LipPressorR"),

            ("LipPuckerL",
             "LipPuckerR"),

            ("LipStretcherL",
             "LipStretcherR"),

            ("LipTightenerL",
             "LipTightenerR"),

            ("LowerLipDepressorL",
             "LowerLipDepressorR"),

            ("NoseWrinklerL",
             "NoseWrinklerR"),

            ("OuterBrowRaiserL",
             "OuterBrowRaiserR"),

            ("UpperLidRaiserL",
             "UpperLidRaiserR"),

            ("UpperLipRaiserL",
             "UpperLipRaiserR"),
        ]

        asymmetry_sum = 0.0

        for left, right in paired_features:

            asymmetry_sum += abs(
                self.face_value(left) -
                self.face_value(right)
            )

        facial_asymmetry = (
            asymmetry_sum /
            len(paired_features)
        )

        # ========================================================
        # GAZE METRICS
        # ========================================================

        # --------------------------------------------------------
        # Gaze velocity statistics
        # --------------------------------------------------------

        recent_velocities = list(
            self.velocities
        )

        if recent_velocities:

            mean_velocity = (
                sum(
                    recent_velocities
                ) /
                len(
                    recent_velocities
                )
            )

            velocity_variability = math.sqrt(
                sum(
                    (
                        velocity -
                        mean_velocity
                    ) ** 2
                    for velocity
                    in recent_velocities
                )
                /
                len(
                    recent_velocities
                )
            )

            high_velocity_fraction = (
                sum(
                    velocity >
                    self.fixation_velocity_threshold
                    for velocity
                    in recent_velocities
                )
                /
                len(
                    recent_velocities
                )
            )

        else:

            mean_velocity = 0.0
            velocity_variability = 0.0
            high_velocity_fraction = 0.0

        # --------------------------------------------------------
        # Fixation metrics
        # --------------------------------------------------------

        if self.fixation_durations:

            mean_fixation_duration = (
                sum(
                    self.fixation_durations
                ) /
                len(
                    self.fixation_durations
                )
            )

            fixation_rate = (
                len(
                    self.fixation_durations
                ) /
                self.gaze_window_seconds
            )

        else:

            mean_fixation_duration = 0.0
            fixation_rate = 0.0

        # --------------------------------------------------------
        # Saccade metrics
        # --------------------------------------------------------

        if self.saccade_amplitudes:

            mean_saccade_amplitude = (
                sum(
                    self.saccade_amplitudes
                ) /
                len(
                    self.saccade_amplitudes
                )
            )

            saccade_rate = (
                len(
                    self.saccade_amplitudes
                ) /
                self.gaze_window_seconds
            )

        else:

            mean_saccade_amplitude = 0.0
            saccade_rate = 0.0

        # --------------------------------------------------------
        # Gaze dispersion
        #
        # RMS distance from the mean gaze location.
        # --------------------------------------------------------

        if len(self.positions) > 1:

            mean_x = (
                sum(
                    position[0]
                    for position
                    in self.positions
                ) /
                len(
                    self.positions
                )
            )

            mean_y = (
                sum(
                    position[1]
                    for position
                    in self.positions
                ) /
                len(
                    self.positions
                )
            )

            gaze_dispersion = math.sqrt(
                sum(
                    (
                        position[0] -
                        mean_x
                    ) ** 2
                    +
                    (
                        position[1] -
                        mean_y
                    ) ** 2
                    for position
                    in self.positions
                )
                /
                len(
                    self.positions
                )
            )

        else:

            gaze_dispersion = 0.0

        # --------------------------------------------------------
        # Gaze entropy
        # --------------------------------------------------------

        gaze_entropy = (
            self.calculate_gaze_entropy()
        )

        # ========================================================
        # BINOCULAR METRICS
        # ========================================================

        # --------------------------------------------------------
        # Mean vergence
        # --------------------------------------------------------

        if self.vergence_history:

            mean_vergence = (
                sum(
                    self.vergence_history
                ) /
                len(
                    self.vergence_history
                )
            )

            # ----------------------------------------------------
            # Vergence variability
            # ----------------------------------------------------

            vergence_variability = math.sqrt(
                sum(
                    (
                        value -
                        mean_vergence
                    ) ** 2
                    for value
                    in self.vergence_history
                )
                /
                len(
                    self.vergence_history
                )
            )

        else:

            mean_vergence = 0.0
            vergence_variability = 0.0

        # ========================================================
        # CURRENT GAZE
        # ========================================================

        left_x = self.latest_gaze[0]
        left_y = self.latest_gaze[1]

        right_x = self.latest_gaze[2]
        right_y = self.latest_gaze[3]

        # Midpoint/cyclopean gaze.
        gaze_x = self.latest_gaze[4]
        gaze_y = self.latest_gaze[5]

        # ========================================================
        # CREATE OUTPUT MESSAGE
        # ========================================================

        output = Float64MultiArray()

        output.data = [

            # ----------------------------------------------------
            # Facial metrics
            # ----------------------------------------------------

            # 0
            eye_closure,

            # 1
            perclos,

            # 2
            float(blink_rate),

            # 3
            mean_blink_duration,

            # 4
            jaw_drop,

            # 5
            facial_activity,

            # 6
            facial_asymmetry,

            # ----------------------------------------------------
            # Gaze metrics
            # ----------------------------------------------------

            # 7
            mean_velocity,

            # 8
            velocity_variability,

            # 9
            high_velocity_fraction,

            # 10
            mean_fixation_duration,

            # 11
            fixation_rate,

            # 12
            mean_saccade_amplitude,

            # 13
            saccade_rate,

            # 14
            gaze_dispersion,

            # 15
            gaze_entropy,

            # ----------------------------------------------------
            # Binocular metrics
            # ----------------------------------------------------

            # 16
            mean_vergence,

            # 17
            vergence_variability,

            # ----------------------------------------------------
            # Raw gaze coordinates
            # ----------------------------------------------------

            # 18
            left_x,

            # 19
            left_y,

            # 20
            right_x,

            # 21
            right_y,

            # ----------------------------------------------------
            # Cyclopean/midpoint gaze
            # ----------------------------------------------------

            # 22
            gaze_x,

            # 23
            gaze_y,
        ]

        # Publish.
        self.publisher.publish(
            output
        )


# =================================================================
# Main
# =================================================================

def main(args=None):

    # Initialize ROS 2.
    rclpy.init(args=args)

    # Create node.
    node = OperatorMetricsNode()

    try:

        # Keep node running.
        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Clean shutdown.
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()