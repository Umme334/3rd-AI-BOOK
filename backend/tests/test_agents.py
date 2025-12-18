import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.agents.agent_manager import AgentManager
from src.agents.base_agent import BaseAgent
from src.agents.coordinator_agent import CoordinatorAgent
from src.agents.retrieval_agent import RetrievalAgent
from src.agents.generation_agent import GenerationAgent
from src.agents.routing_agent import RoutingAgent


class TestAgentManager:
    """Test cases for Agent Manager"""

    @pytest.fixture
    def agent_manager(self):
        return AgentManager()

    @pytest.mark.asyncio
    async def test_initialize_agents(self, agent_manager):
        """Test initializing all RAG agents"""
        agent_manager.initialize_agents()

        assert "retrieval_agent" in agent_manager.agents
        assert "generation_agent" in agent_manager.agents
        assert "routing_agent" in agent_manager.agents
        assert "coordinator_agent" in agent_manager.agents

        assert isinstance(agent_manager.agents["retrieval_agent"], RetrievalAgent)
        assert isinstance(agent_manager.agents["generation_agent"], GenerationAgent)
        assert isinstance(agent_manager.agents["routing_agent"], RoutingAgent)
        assert isinstance(agent_manager.agents["coordinator_agent"], CoordinatorAgent)

    @pytest.mark.asyncio
    async def test_get_agent(self, agent_manager):
        """Test getting a specific agent by ID"""
        agent_manager.initialize_agents()

        agent = agent_manager.get_agent("retrieval_agent")
        assert agent is not None
        assert isinstance(agent, RetrievalAgent)

    @pytest.mark.asyncio
    async def test_process_rag_query(self, agent_manager):
        """Test processing a complete RAG query through the agent system"""
        agent_manager.initialize_agents()

        # Mock the coordinator's process_message method
        coordinator = agent_manager.get_agent("coordinator_agent")
        with patch.object(coordinator, 'process_message', return_value=MagicMock(content="Test response")):
            result = await agent_manager.process_rag_query(
                query="Test query",
                textbook_id="test-book",
                session_id="test-session",
                selected_text="test selected text"
            )

            assert result is not None


class TestBaseAgent:
    """Test cases for Base Agent"""

    @pytest.fixture
    def base_agent(self):
        class TestAgent(BaseAgent):
            def __init__(self):
                super().__init__("test_agent", "Test Agent")

            async def process_message(self, message):
                return message

        return TestAgent()

    def test_agent_initialization(self, base_agent):
        """Test agent initialization"""
        assert base_agent.agent_id == "test_agent"
        assert base_agent.name == "Test Agent"
        assert base_agent.subscribers == set()
        assert base_agent.message_queue is not None


class TestCoordinatorAgent:
    """Test cases for Coordinator Agent"""

    @pytest.fixture
    def coordinator_agent(self):
        routing_agent_mock = MagicMock()
        return CoordinatorAgent(
            agent_id="test_coordinator",
            name="Test Coordinator",
            routing_agent=routing_agent_mock
        )

    @pytest.mark.asyncio
    async def test_process_message(self, coordinator_agent):
        """Test processing a message in coordinator agent"""
        from src.agents.base_agent import AgentMessage

        message = AgentMessage(
            sender="user",
            recipient="coordinator_agent",
            content="Test query",
            metadata={
                "textbook_id": "test-book",
                "session_id": "test-session",
                "selected_text": "test text",
                "top_k": 5
            },
            message_type="rag_query"
        )

        result = await coordinator_agent.process_message(message)
        # Coordinator should handle the message appropriately
        assert result is not None


class TestRetrievalAgent:
    """Test cases for Retrieval Agent"""

    @pytest.fixture
    def retrieval_agent(self):
        return RetrievalAgent(
            agent_id="test_retrieval",
            name="Test Retrieval"
        )

    @pytest.mark.asyncio
    async def test_process_message(self, retrieval_agent):
        """Test processing a message in retrieval agent"""
        from src.agents.base_agent import AgentMessage

        message = AgentMessage(
            sender="coordinator",
            recipient="retrieval_agent",
            content="Test query",
            metadata={
                "textbook_id": "test-book",
                "query": "Test query",
                "top_k": 5
            },
            message_type="retrieval_request"
        )

        # Mock the RAG service's search_context method
        with patch.object(retrieval_agent.rag_service, 'search_context', return_value=[
            {
                "id": "chunk-1",
                "content": "Test content",
                "source": "test-source",
                "chapter": "Test Chapter",
                "section": "Test Section",
                "similarity_score": 0.9
            }
        ]):
            result = await retrieval_agent.process_message(message)
            assert result is not None


class TestGenerationAgent:
    """Test cases for Generation Agent"""

    @pytest.fixture
    def generation_agent(self):
        return GenerationAgent(
            agent_id="test_generation",
            name="Test Generation"
        )

    @pytest.mark.asyncio
    async def test_process_message(self, generation_agent):
        """Test processing a message in generation agent"""
        from src.agents.base_agent import AgentMessage

        message = AgentMessage(
            sender="coordinator",
            recipient="generation_agent",
            content="Test query",
            metadata={
                "query": "Test query",
                "context_chunks": [
                    {
                        "content": "Test context content",
                        "source": "test-source",
                        "chapter": "Test Chapter",
                        "section": "Test Section",
                        "similarity_score": 0.9
                    }
                ]
            },
            message_type="generation_request"
        )

        # Mock the RAG service's _generate_response method
        with patch.object(generation_agent.rag_service, '_generate_response', return_value="Generated response"):
            result = await generation_agent.process_message(message)
            assert result is not None


class TestRoutingAgent:
    """Test cases for Routing Agent"""

    @pytest.fixture
    def routing_agent(self):
        agents_dict = {
            "retrieval_agent": RetrievalAgent("retrieval_agent", "Retrieval Agent"),
            "generation_agent": GenerationAgent("generation_agent", "Generation Agent")
        }
        return RoutingAgent(
            agent_id="test_routing",
            name="Test Routing",
            agents=agents_dict
        )

    @pytest.mark.asyncio
    async def test_route_message(self, routing_agent):
        """Test routing a message to the appropriate agent"""
        from src.agents.base_agent import AgentMessage

        message = AgentMessage(
            sender="coordinator",
            recipient="retrieval_agent",
            content="Test query",
            metadata={},
            message_type="retrieval_request"
        )

        # The routing agent should be able to route messages
        result = await routing_agent.process_message(message)
        assert result is not None