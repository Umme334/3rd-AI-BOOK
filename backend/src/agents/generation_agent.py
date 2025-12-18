from typing import Dict, Any, Optional
from ..services.openai_client import OpenAIClient
from .base_agent import BaseAgent, AgentMessage


class GenerationAgent(BaseAgent):
    """Agent responsible for generating responses based on context"""

    def __init__(self, agent_id: str, name: str):
        super().__init__(agent_id, name)
        self.openai_client = OpenAIClient()

    async def process_message(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process generation requests"""
        if message.message_type == "generate_response":
            return await self._handle_generate_response(message)
        elif message.message_type == "summarize_content":
            return await self._handle_summarize_content(message)
        else:
            return await self._handle_default(message)

    async def _handle_generate_response(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle response generation requests"""
        try:
            query = message.content
            context = message.metadata.get("context", "")
            selected_text = message.metadata.get("selected_text", "")

            # Use the context and selected text to generate response
            full_context = context
            if selected_text:
                full_context = f"Selected Text: {selected_text}\n\nContext: {context}"

            response = await self.openai_client.generate_chat_response(
                query=query,
                context=full_context
            )

            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=response,
                metadata={"query": query, "context_used": bool(full_context)},
                message_type="generated_response"
            )
        except Exception as e:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Error during response generation: {str(e)}",
                metadata={"error": str(e)},
                message_type="error"
            )

    async def _handle_summarize_content(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle content summarization requests"""
        try:
            content = message.content
            max_length = message.metadata.get("max_length", 200)

            # Create a summary prompt
            summary_prompt = f"Please provide a concise summary of the following content in no more than {max_length} words: {content}"

            summary = await self.openai_client.generate_chat_response(
                query=summary_prompt,
                context=""
            )

            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=summary,
                metadata={"original_length": len(content), "summary_length": len(summary)},
                message_type="summary_generated"
            )
        except Exception as e:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Error during summarization: {str(e)}",
                metadata={"error": str(e)},
                message_type="error"
            )

    async def _handle_default(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle default message"""
        return AgentMessage(
            sender=self.agent_id,
            recipient=message.sender,
            content="Generation agent received message",
            metadata={"handled_by": self.name},
            message_type="acknowledged"
        )