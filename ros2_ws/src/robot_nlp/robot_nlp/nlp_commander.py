#!/usr/bin/env python3
"""
Natural language command processor using OpenAI ChatGPT
Converts English commands to ROS 2 action sequences
"""
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import NLPCommand, CraneCommand, SetMode
from geometry_msgs.msg import PoseStamped
import os
from dotenv import load_dotenv

# Try to import openai
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

class NLPCommander(Node):
    def __init__(self):
        super().__init__('nlp_commander')
        
        # Load environment variables
        load_dotenv()
        api_key = os.getenv('OPENAI_API_KEY')
        
        if not OPENAI_AVAILABLE:
            self.get_logger().error('OpenAI not installed: pip install openai')
            self.enabled = False
        elif not api_key or api_key == 'your-openai-api-key-here':
            self.get_logger().error('OpenAI API key not set in .env file')
            self.enabled = False
        else:
            openai.api_key = api_key
            self.enabled = True
            self.get_logger().info('NLP commander initialized with OpenAI')
        
        # Service
        self.nlp_srv = self.create_service(
            NLPCommand, '/nlp/command', self.process_command)
        
        # Service clients (for executing commands)
        self.crane_client = self.create_client(CraneCommand, '/crane/command')
        self.mode_client = self.create_client(SetMode, '/mode/set')
    
    def process_command(self, request, response):
        """Process natural language command"""
        if not self.enabled:
            response.success = False
            response.error_message = "NLP not available - check API key"
            return response
        
        command_text = request.command_text
        self.get_logger().info(f'Processing: "{command_text}"')
        
        try:
            # Call ChatGPT to interpret command
            interpretation = self.interpret_command(command_text)
            
            # Parse and execute
            actions = self.parse_actions(interpretation)
            
            response.success = True
            response.interpretation = interpretation
            response.planned_actions = actions
            response.error_message = ""
            
            # Execute actions (async)
            self.execute_actions(actions)
            
        except Exception as e:
            self.get_logger().error(f'NLP error: {e}')
            response.success = False
            response.error_message = str(e)
        
        return response
    
    def interpret_command(self, text):
        """Use ChatGPT to interpret command"""
        system_prompt = """You are a robot command interpreter. Convert natural language 
commands into simple action sequences for a differential drive robot with a crane.

Available actions:
- MOVE_TO(location): Navigate to named location
- CRANE_HOME: Home the crane
- CRANE_STOW: Stow crane for travel
- CRANE_PICK: Lower crane to pick position
- CRANE_PLACE: Raise crane to place position
- WAIT(seconds): Wait for specified time

Example:
Input: "go to bin A, pick it up, deliver to zone B, then return home"
Output: MOVE_TO(bin_A), CRANE_PICK, WAIT(2), CRANE_STOW, MOVE_TO(zone_B), 
CRANE_PLACE, WAIT(2), CRANE_STOW, MOVE_TO(home)

Respond with only the action sequence, no explanation."""

        try:
            completion = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=200,
                temperature=0.3
            )
            return completion.choices[0].message.content.strip()
        except Exception as e:
            self.get_logger().error(f'OpenAI API error: {e}')
            raise
    
    def parse_actions(self, interpretation):
        """Parse action sequence from ChatGPT response"""
        actions = []
        for part in interpretation.split(','):
            action = part.strip()
            if action:
                actions.append(action)
        return actions
    
    def execute_actions(self, actions):
        """Execute action sequence (simplified)"""
        self.get_logger().info(f'Executing {len(actions)} actions...')
        for action in actions:
            self.get_logger().info(f'  → {action}')
            # Actual execution would call appropriate services/actions
            # This is a placeholder showing the architecture

def main(args=None):
    rclpy.init(args=args)
    node = NLPCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
