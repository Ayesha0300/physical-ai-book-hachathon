import requests
import time
from urllib.parse import urlparse

def check_url_status(url, timeout=10):
    """Check if a URL is accessible"""
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        if response.status_code == 200:
            return True
        else:
            # Some sites don't support HEAD, try GET
            response = requests.get(url, timeout=timeout, allow_redirects=True)
            return response.status_code == 200
    except:
        return False

# Read all URLs from sitemap
sitemap_url = "https://physical-ai-book-hachathon.vercel.app/sitemap.xml"
response = requests.get(sitemap_url)
content = response.text

# Extract URLs from sitemap XML
import xml.etree.ElementTree as ET
root = ET.fromstring(content)

# Handle XML namespaces
namespaces = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

# Try to find URL elements with namespace
urls = []
url_elements = root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url')

# If no elements found with namespace, try without namespace
if not url_elements:
    url_elements = root.findall('.//url')

for url_elem in url_elements:
    # Look for loc element with namespace
    loc_elem = url_elem.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
    if loc_elem is None:
        # Try without namespace
        loc_elem = url_elem.find('loc')

    if loc_elem is not None:
        urls.append(loc_elem.text)

print(f"Found {len(urls)} URLs in sitemap")

# Check which URLs are accessible
accessible_urls = []
for i, url in enumerate(urls):
    print(f"Checking {i+1}/{len(urls)}: {url}", end="... ")
    if check_url_status(url):
        accessible_urls.append(url)
        print("OK")
    else:
        print("404")
    time.sleep(0.1)  # Be respectful to the server

print(f"\nTotal accessible URLs: {len(accessible_urls)}")
print("Writing accessible URLs to file...")

with open('accessible_urls_comprehensive.txt', 'w') as f:
    for url in accessible_urls:
        f.write(url + '\n')

print("Done! Check accessible_urls_comprehensive.txt for the list of accessible URLs.")