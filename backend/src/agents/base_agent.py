from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
import asyncio
from dataclasses import dataclass


@dataclass
class AgentMessage:
    """Message structure for communication between agents"""
    sender: str
    recipient: str
    content: str
    metadata: Dict[str, Any]
    message_type: str = "info"


class BaseAgent(ABC):
    """Base class for all agents in the system"""

    def __init__(self, agent_id: str, name: str):
        self.agent_id = agent_id
        self.name = name
        self.message_queue = asyncio.Queue()
        self.subscribers = []

    @abstractmethod
    async def process_message(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process an incoming message and return a response"""
        pass

    async def send_message(self, recipient: str, content: str, metadata: Dict[str, Any] = None) -> None:
        """Send a message to another agent"""
        if metadata is None:
            metadata = {}

        message = AgentMessage(
            sender=self.agent_id,
            recipient=recipient,
            content=content,
            metadata=metadata
        )

        # Add to global message queue (in a real system, this would be a shared queue)
        await self.message_queue.put(message)

    def subscribe(self, agent):
        """Subscribe to messages from this agent"""
        self.subscribers.append(agent)

    async def run(self):
        """Main agent loop"""
        while True:
            try:
                message = await self.message_queue.get()
                if message.recipient == self.agent_id:
                    response = await self.process_message(message)
                    if response:
                        # Forward response to subscribers
                        for subscriber in self.subscribers:
                            await subscriber.receive_message(response)
            except Exception as e:
                print(f"Error in agent {self.name}: {e}")

    async def receive_message(self, message: AgentMessage):
        """Receive a message from another agent"""
        await self.message_queue.put(message)