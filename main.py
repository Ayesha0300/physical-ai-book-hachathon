from backend.agent import RAGAgent
from backend.config import Config
import sys

def main():
    """Main entry point for the RAG Agent CLI."""
    print("RAG Agent with Gemini - Starting...")
    
    try:
        # Validate configuration first
        Config.validate()
        print("Configuration validated successfully")
        
        # Create the agent
        agent = RAGAgent(answer_only_mode=True)
        print("RAG Agent initialized successfully")
        
        # Check health of services
        health_status = agent.check_health()
        print(f"Service health: {health_status}")
        
        if not health_status['overall']:
            print("Warning: Some services are not healthy")
        
        # Simple test query
        print("\nTesting with a sample query...")
        test_query = "What is the purpose of this RAG agent?"
        response = agent.query(test_query)
        print(f"Query: {test_query}")
        print(f"Response: {response}")
        
        # Interactive mode
        print("\nEntering interactive mode. Type 'quit' to exit.")
        while True:
            user_input = input("\nYour query: ")
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            
            response = agent.query(user_input)
            print(f"Response: {response}")
            
    except Exception as e:
        print(f"Error during execution: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
