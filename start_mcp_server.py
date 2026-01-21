#!/usr/bin/env python3
"""
Startup script for the Context7 RAG Agent MCP Server
"""

import subprocess
import sys
import os

def install_dependencies():
    """Install required dependencies."""
    print("Installing required dependencies...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "uvicorn", "fastapi"])


def run_mcp_server():
    """Run the MCP server."""
    print("Starting Context7 RAG Agent MCP Server...")

    # Change to backend directory
    backend_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "backend")
    os.chdir(backend_dir)

    # Run the server
    subprocess.run([sys.executable, "mcp_server.py"])


if __name__ == "__main__":
    print("Context7 RAG Agent MCP Server Startup")
    print("=====================================")

    # Install dependencies if needed
    try:
        install_dependencies()
    except Exception as e:
        print(f"Could not install dependencies: {e}")
        print("Make sure you have pip and the required packages installed.")
        sys.exit(1)

    # Run the server
    run_mcp_server()