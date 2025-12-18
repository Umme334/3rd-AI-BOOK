from typing import Dict, Any, Optional
from .base_agent import BaseAgent, AgentMessage


class RoutingAgent(BaseAgent):
    """Agent responsible for routing messages to appropriate agents"""

    def __init__(self, agent_id: str, name: str, agents: Dict[str, BaseAgent]):
        super().__init__(agent_id, name)
        self.agents = agents  # Dictionary of available agents

    async def process_message(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process and route messages to appropriate agents"""
        if message.message_type == "route_request":
            return await self._handle_route_request(message)
        elif message.message_type == "register_agent":
            return await self._handle_register_agent(message)
        else:
            # Auto-route based on message type
            return await self._auto_route(message)

    async def _handle_route_request(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle explicit routing requests"""
        target_agent_id = message.metadata.get("target_agent")

        if target_agent_id in self.agents:
            target_agent = self.agents[target_agent_id]
            return await target_agent.process_message(message)
        else:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Target agent {target_agent_id} not found",
                metadata={"available_agents": list(self.agents.keys())},
                message_type="error"
            )

    async def _auto_route(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Auto-route messages based on their content and type"""
        # Determine target agent based on message type
        if message.message_type in ["retrieve_content", "index_content"]:
            target_agent = self.agents.get("retrieval_agent")
        elif message.message_type in ["generate_response", "summarize_content"]:
            target_agent = self.agents.get("generation_agent")
        else:
            # Default to coordinator or return error
            target_agent = self.agents.get("coordinator_agent")

        if target_agent:
            # Forward the message to the appropriate agent
            result = await target_agent.process_message(message)
            # Update the recipient to original sender
            if result:
                result.recipient = message.sender
            return result
        else:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content="No suitable agent found for this request",
                metadata={"message_type": message.message_type},
                message_type="error"
            )

    async def _handle_register_agent(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle agent registration requests"""
        agent_id = message.metadata.get("agent_id")
        agent = message.metadata.get("agent")

        if agent_id and agent:
            self.agents[agent_id] = agent
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Agent {agent_id} registered successfully",
                metadata={"registered_agent": agent_id},
                message_type="registration_success"
            )
        else:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content="Agent registration failed: missing agent_id or agent",
                metadata={"error": "missing_parameters"},
                message_type="error"
            )

    def add_agent(self, agent_id: str, agent: BaseAgent):
        """Add an agent to the routing system"""
        self.agents[agent_id] = agent

    def get_agent(self, agent_id: str) -> Optional[BaseAgent]:
        """Get an agent by ID"""
        return self.agents.get(agent_id)