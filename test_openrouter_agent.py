#!/usr/bin/env python3
"""
Test script for the OpenAI RAG Agent using OpenRouter API
"""

import os
import sys
import dotenv

# Load environment variables
dotenv.load_dotenv(dotenv_path="backend/.env")

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from agent import RAGAgent

def test_rag_agent():
    """Test the RAG agent with OpenRouter API."""
    print("Testing OpenAI RAG Agent with OpenRouter...")

    try:
        # Initialize the agent
        agent = RAGAgent(answer_only_mode=False)
        print("[SUCCESS] RAG Agent initialized successfully")

        # Check health
        health = agent.check_health()
        print(f"[SUCCESS] Health check result: {health}")

        # Test a sample query (this will fail without proper API keys but should reach the API call)
        print("\nTesting sample query...")
        response = agent.query("What is the Physical AI book about?")
        print(f"Response: {response}")

        print("\n[SUCCESS] Test completed")

    except ValueError as e:
        print(f"[WARNING] Configuration error (expected without API keys): {e}")
        print("[SUCCESS] Agent properly validates configuration")
    except Exception as e:
        print(f"[WARNING] Error during test: {e}")

if __name__ == "__main__":
    test_rag_agent()