import requests
import time

# Test some potential URL patterns that might exist
base_url = "https://physical-ai-book-hachathon.vercel.app"

# Test different potential paths
test_paths = [
    "/",
    "/docs",
    "/modules",
    "/capstone",
    "/chatbot",
    "/hardware",
    "/intro",
    "/quickstart",
    "/vla",
    "/module1-ros2",
    "/module2-digital-twin",
    "/module3-isaac",
    "/docs/intro",
    "/docs/hardware",
    "/docs/quickstart",
    "/docs/vla",
    "/docs/module1-ros2",
    "/docs/module2-digital-twin",
    "/docs/module3-isaac"
]

accessible_urls = []

for path in test_paths:
    url = base_url + path
    print(f"Testing {url}...", end=" ")
    try:
        response = requests.get(url, timeout=10)
        if response.status_code == 200:
            print("OK")
            accessible_urls.append(url)
        else:
            print("404")
    except Exception as e:
        print(f"Error: {e}")

    time.sleep(0.1)  # Be respectful to the server

print(f"\nFound {len(accessible_urls)} accessible URLs:")
for url in accessible_urls:
    print(f"  - {url}")

# Also write to file
with open('expanded_accessible_urls.txt', 'w') as f:
    for url in accessible_urls:
        f.write(url + '\n')

print("\nSaved to expanded_accessible_urls.txt")