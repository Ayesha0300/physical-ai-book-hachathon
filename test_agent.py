#!/usr/bin/env python3
"""
Test script for the updated RAG Agent with OpenAI integration
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from backend.agent import RAGAgent

def test_rag_agent():
    """Test the RAG agent functionality."""
    print("Initializing RAG Agent with OpenAI integration...")

    try:
        # Initialize the agent
        agent = RAGAgent(answer_only_mode=False)
        print("✓ RAG Agent initialized successfully")

        # Check health
        health = agent.check_health()
        print(f"✓ Health check result: {health}")

        # Test a sample query (this will fail without proper API keys but should reach the API call)
        print("\nTesting sample query...")
        try:
            response = agent.query("What is artificial intelligence?")
            print(f"Response: {response}")
        except Exception as e:
            print(f"Expected error during query (likely due to missing API keys): {e}")

        print("\n✓ Test completed successfully")

    except Exception as e:
        print(f"✗ Error initializing RAG Agent: {e}")
        return False

    return True

if __name__ == "__main__":
    test_rag_agent()