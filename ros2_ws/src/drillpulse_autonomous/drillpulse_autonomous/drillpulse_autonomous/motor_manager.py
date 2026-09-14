from std_msgs.msg import String, Int32
from drillpulse_autonomous.utils import log_colored

class MotorManager:
    """
    Translates high-level navigation decisions into exact DrillPulse motor bridge commands.
    Publishes string commands to '/rover/command' reusing the existing WiFi/serial protocol.
    """
    COMMAND_MAP = {
        "FORWARD": "CMD,0,1",
        "BACKWARD": "CMD,0,-1",
        "REVERSE": "CMD,0,-1",
        "LEFT": "CMD,-1,0",
        "TURN_LEFT": "CMD,-1,0",
        "RIGHT": "CMD,1,0",
        "TURN_RIGHT": "CMD,1,0",
        "SLOW": "CMD,0,1",
        "STOP": "CMD,0,0",
        "EMERGENCY_STOP": "CMD,0,0"
    }

    def __init__(self, node, topic="/rover/command", speed_topic="/speed_mode"):
        self.node = node
        self.topic = topic
        self.speed_topic = speed_topic
        
        self.last_command = None
        self.current_speed_mode = 1

        if self.node is not None:
            self.command_pub = self.node.create_publisher(String, self.topic, 10)
            self.speed_pub = self.node.create_publisher(Int32, self.speed_topic, 10)
            log_colored("MOTOR", f"Motor manager initialized. Publishing to: {self.topic}", self.node.get_logger())

    def send_movement(self, direction_intent: str, force: bool = False):
        """
        Sends movement command ('FORWARD', 'REVERSE', 'TURN_LEFT', 'TURN_RIGHT', 'SLOW', 'STOP').
        Only publishes if the command has changed or force=True.
        """
        intent = direction_intent.upper()
        if intent == "SLOW":
            self.set_speed(1)  # Set speed mode to SLOW (1)
            
        raw_cmd = self.COMMAND_MAP.get(intent, "CMD,0,0")
        
        if force or raw_cmd != self.last_command:
            msg = String()
            msg.data = raw_cmd
            
            if self.node is not None:
                self.command_pub.publish(msg)
                log_colored("MOTOR", f"TX -> {self.topic}: {raw_cmd} ({direction_intent.upper()})", self.node.get_logger())
            
            self.last_command = raw_cmd

    def set_speed(self, mode: int):
        """
        Sets speed level (1: SLOW, 2: NORMAL, 3: TURBO).
        Publishes both speed mode topic and SPEED,n string command.
        """
        mode = max(1, min(3, mode))
        if mode != self.current_speed_mode:
            self.current_speed_mode = mode
            
            # Send speed command to Arduino bridge
            msg_str = String()
            msg_str.data = f"SPEED,{mode}"
            
            msg_int = Int32()
            msg_int.data = mode
            
            if self.node is not None:
                self.command_pub.publish(msg_str)
                self.speed_pub.publish(msg_int)
                log_colored("MOTOR", f"Speed updated to level {mode}", self.node.get_logger())

    def emergency_stop(self):
        """Instantly halts all motor movement."""
        self.send_movement("EMERGENCY_STOP", force=True)
        log_colored("MOTOR", "EMERGENCY STOP EXECUTED!", getattr(self.node, 'get_logger', lambda: None)())
