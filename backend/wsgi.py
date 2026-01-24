"""
WSGI entry point for Vercel deployment
"""

import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import app

# Vercel expects the application to be named 'application'
application = app