from typing import Dict, Any, Optional
from .base_agent import BaseAgent, AgentMessage


class CoordinatorAgent(BaseAgent):
    """Agent responsible for coordinating the overall RAG workflow"""

    def __init__(self, agent_id: str, name: str, routing_agent: 'RoutingAgent'):
        super().__init__(agent_id, name)
        self.routing_agent = routing_agent
        self.active_sessions = {}  # Track active RAG sessions

    async def process_message(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process coordination requests"""
        if message.message_type == "rag_query":
            return await self._handle_rag_query(message)
        elif message.message_type == "start_session":
            return await self._handle_start_session(message)
        elif message.message_type == "end_session":
            return await self._handle_end_session(message)
        else:
            return await self._handle_default(message)

    async def _handle_rag_query(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle a complete RAG query workflow"""
        try:
            query = message.content
            textbook_id = message.metadata.get("textbook_id")
            session_id = message.metadata.get("session_id")
            selected_text = message.metadata.get("selected_text", "")

            # Step 1: Retrieve relevant content
            retrieval_request = AgentMessage(
                sender=self.agent_id,
                recipient="retrieval_agent",
                content=query,
                metadata={
                    "textbook_id": textbook_id,
                    "top_k": message.metadata.get("top_k", 5)
                },
                message_type="retrieve_content"
            )

            retrieval_result = await self.routing_agent.process_message(retrieval_request)

            if retrieval_result and retrieval_result.message_type == "retrieval_results":
                context_chunks = retrieval_result.metadata.get("results", [])

                # Format context from retrieved chunks
                context = "\n\n".join([chunk.get("content", "") for chunk in context_chunks])

                # Step 2: Generate response using the context
                generation_request = AgentMessage(
                    sender=self.agent_id,
                    recipient="generation_agent",
                    content=query,
                    metadata={
                        "context": context,
                        "selected_text": selected_text
                    },
                    message_type="generate_response"
                )

                generation_result = await self.routing_agent.process_message(generation_request)

                if generation_result and generation_result.message_type == "generated_response":
                    # Return the final response
                    return AgentMessage(
                        sender=self.agent_id,
                        recipient=message.sender,
                        content=generation_result.content,
                        metadata={
                            "context_sources": [chunk.get("id") for chunk in context_chunks],
                            "query": query,
                            "session_id": session_id
                        },
                        message_type="rag_response"
                    )
                else:
                    return generation_result
            else:
                # If retrieval failed, still try to generate a response
                generation_request = AgentMessage(
                    sender=self.agent_id,
                    recipient="generation_agent",
                    content=query,
                    metadata={
                        "context": "",
                        "selected_text": selected_text
                    },
                    message_type="generate_response"
                )

                return await self.routing_agent.process_message(generation_request)

        except Exception as e:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Error in RAG workflow: {str(e)}",
                metadata={"error": str(e)},
                message_type="error"
            )

    async def _handle_start_session(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle session start requests"""
        session_id = message.metadata.get("session_id")
        textbook_id = message.metadata.get("textbook_id")

        if session_id:
            self.active_sessions[session_id] = {
                "textbook_id": textbook_id,
                "start_time": message.metadata.get("timestamp"),
                "queries": []
            }

            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Session {session_id} started successfully",
                metadata={"session_id": session_id, "textbook_id": textbook_id},
                message_type="session_started"
            )
        else:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content="Session start failed: missing session_id",
                metadata={"error": "missing_session_id"},
                message_type="error"
            )

    async def _handle_end_session(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle session end requests"""
        session_id = message.metadata.get("session_id")

        if session_id and session_id in self.active_sessions:
            session_data = self.active_sessions.pop(session_id)
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Session {session_id} ended successfully",
                metadata={"session_id": session_id, "queries_count": len(session_data["queries"])},
                message_type="session_ended"
            )
        else:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Session {session_id} not found or already ended",
                metadata={"session_id": session_id},
                message_type="error"
            )

    async def _handle_default(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle default message"""
        return AgentMessage(
            sender=self.agent_id,
            recipient=message.sender,
            content="Coordinator agent received message",
            metadata={"handled_by": self.name},
            message_type="acknowledged"
        )

    def get_session_data(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get data for a specific session"""
        return self.active_sessions.get(session_id)