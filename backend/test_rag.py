from src.services.chatbot.rag_service import RAGService
import asyncio

async def test_rag():
    try:
        rag = RAGService()
        print("RAG Service initialized successfully")

        # Test creating a session
        session = await rag.create_chatbot_session('test-book')
        print(f"Session created: {session.id}")

        # Test processing a query
        response = await rag.process_query(session, 'Hello', 'test selected text')
        print(f"Response: {response[:100]}...")

        print("Basic RAG functionality test completed successfully")
    except Exception as e:
        print(f"Error in RAG test: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_rag())