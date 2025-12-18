from typing import Dict, List
import asyncio
from .retrieval_agent import RetrievalAgent
from .generation_agent import GenerationAgent
from .routing_agent import RoutingAgent
from .coordinator_agent import CoordinatorAgent


class AgentManager:
    """Manages the lifecycle and coordination of all RAG agents"""

    def __init__(self):
        self.agents: Dict[str, 'BaseAgent'] = {}
        self.running_tasks: List[asyncio.Task] = []

    def initialize_agents(self):
        """Initialize all RAG agents"""
        # Create individual agents
        retrieval_agent = RetrievalAgent(
            agent_id="retrieval_agent",
            name="Retrieval Agent"
        )

        generation_agent = GenerationAgent(
            agent_id="generation_agent",
            name="Generation Agent"
        )

        # Create routing agent with initial agent mapping
        agents_dict = {
            "retrieval_agent": retrieval_agent,
            "generation_agent": generation_agent
        }

        routing_agent = RoutingAgent(
            agent_id="routing_agent",
            name="Routing Agent",
            agents=agents_dict
        )

        coordinator_agent = CoordinatorAgent(
            agent_id="coordinator_agent",
            name="Coordinator Agent",
            routing_agent=routing_agent
        )

        # Add the coordinator to the routing agent
        routing_agent.add_agent("coordinator_agent", coordinator_agent)

        # Store all agents
        self.agents = {
            "retrieval_agent": retrieval_agent,
            "generation_agent": generation_agent,
            "routing_agent": routing_agent,
            "coordinator_agent": coordinator_agent
        }

        # Set up communication between agents
        self._setup_agent_communication()

    def _setup_agent_communication(self):
        """Set up communication channels between agents"""
        # All agents can potentially communicate with each other
        for agent in self.agents.values():
            for other_agent in self.agents.values():
                if agent.agent_id != other_agent.agent_id:
                    agent.subscribe(other_agent)

    def start_agents(self):
        """Start all agent event loops"""
        for agent_name, agent in self.agents.items():
            task = asyncio.create_task(agent.run())
            self.running_tasks.append(task)

    def stop_agents(self):
        """Stop all running agent tasks"""
        for task in self.running_tasks:
            task.cancel()

    def get_agent(self, agent_id: str):
        """Get a specific agent by ID"""
        return self.agents.get(agent_id)

    def send_message(self, sender_id: str, recipient_id: str, content: str, metadata: dict = None, message_type: str = "info"):
        """Send a message between agents"""
        sender = self.get_agent(sender_id)
        recipient = self.get_agent(recipient_id)

        if sender and recipient and metadata:
            message = sender.__class__.__dict__.get('_create_message', lambda s, r, c, m, mt: None)(
                sender.agent_id, recipient.agent_id, content, metadata, message_type
            )
            if message:
                # This is a simplified approach - in a real system we'd have a shared message queue
                pass

    async def process_rag_query(self, query: str, textbook_id: str, session_id: str = None, selected_text: str = ""):
        """Process a complete RAG query through the agent system"""
        coordinator = self.get_agent("coordinator_agent")

        if coordinator:
            from .base_agent import AgentMessage

            message = AgentMessage(
                sender="user",
                recipient="coordinator_agent",
                content=query,
                metadata={
                    "textbook_id": textbook_id,
                    "session_id": session_id,
                    "selected_text": selected_text,
                    "top_k": 5
                },
                message_type="rag_query"
            )

            # Process the message through the coordinator
            result = await coordinator.process_message(message)
            return result
        else:
            return None

    def get_agent_status(self) -> Dict[str, str]:
        """Get status of all agents"""
        return {agent_id: "initialized" for agent_id in self.agents.keys()}