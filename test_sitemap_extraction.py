#!/usr/bin/env python3
"""
Test script to verify sitemap content extraction without using APIs.

This script will test the sitemap extraction and content scraping functionality
without requiring API keys for Cohere and Qdrant.
"""
import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / 'rag-pipeline' / 'src'))

from src.content_extractor.web_scraper import WebScraper
from src.utils.logger import app_logger as logger
import time

def test_sitemap_extraction():
    """Test sitemap extraction and content scraping."""
    print("Testing Sitemap Extraction and Content Scraping")
    print("=" * 50)

    # Initialize the web scraper
    scraper = WebScraper()

    # Test sitemap URL
    sitemap_url = "https://physical-ai-book-umber-seven.vercel.app/sitemap.xml"
    print(f"Fetching sitemap from: {sitemap_url}")

    # Extract URLs from sitemap
    urls = scraper.extract_urls_from_sitemap(sitemap_url)
    print(f"Found {len(urls)} URLs in sitemap")

    if not urls:
        print("ERROR: No URLs found in sitemap!")
        return

    # Test a few sample URLs to verify they can be accessed
    print(f"\nTesting first few URLs from sitemap:")
    test_urls = urls[:10]  # Test first 10 URLs
    successful = 0
    failed = 0

    for i, url in enumerate(test_urls, 1):
        print(f"  {i}. Testing: {url}")
        content = scraper.fetch_content(url, max_retries=3)
        if content:
            print(f"     ✓ Successfully fetched content ({len(content)} chars)")
            successful += 1
            # Parse the content to ensure it's valid HTML
            parsed = scraper.parse_html_content(content, url)
            if parsed['content'] and len(parsed['content']) > 10:  # Ensure we got meaningful content
                print(f"     ✓ Content parsed successfully (title: '{parsed['metadata']['title'][:50]}...')")
            else:
                print(f"     ⚠ Content parsing resulted in empty content")
        else:
            print(f"     ✗ Failed to fetch content")
            failed += 1

        # Small delay to be respectful to the server
        time.sleep(0.5)

    print(f"\nTest Results:")
    print(f"  Total URLs tested: {len(test_urls)}")
    print(f"  Successful: {successful}")
    print(f"  Failed: {failed}")
    print(f"  Success rate: {successful/len(test_urls)*100:.1f}% if {len(test_urls)} > 0 else 0%")

    # Show some statistics about the full sitemap
    print(f"\nFull sitemap statistics:")
    print(f"  Total URLs in sitemap: {len(urls)}")
    print(f"  Sample URLs:")
    for i, url in enumerate(urls[:5], 1):  # Show first 5 URLs
        print(f"    {i}. {url}")
    if len(urls) > 5:
        print(f"    ... and {len(urls) - 5} more URLs")

if __name__ == "__main__":
    test_sitemap_extraction()