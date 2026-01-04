# Docusaurus Build Validation Script

This script validates that the new Module 2 content builds correctly with Docusaurus.

## Prerequisites
- Node.js and npm installed
- Docusaurus project dependencies installed (run `npm install` in project root)

## Validation Steps

### 1. Build Validation
```bash
npm run build
```
This command builds the static site and will fail if there are any build errors with the new content.

### 2. Local Server Test
```bash
npm run start
```
This command starts a local development server to manually verify that:
- All new pages load correctly
- Navigation works properly
- Links function as expected
- Content displays properly formatted

### 3. Link Validation
```bash
# Check for broken links in the build
npm run build && npx linkinator build --recurse --verbosity error
```
This checks for any broken internal or external links in the built site.

### 4. Automated Testing (if available)
```bash
npm test
```
Run any existing automated tests to ensure new content doesn't break existing functionality.

## Specific Checks for Module 2 Content

### Navigation Validation
- [ ] Module 2 appears in sidebar navigation
- [ ] All 7 chapters are listed in correct order
- [ ] Links to all chapter pages work correctly
- [ ] Back-to-navigation links function properly

### Content Validation
- [ ] All chapter pages load without errors
- [ ] Markdown formatting renders correctly
- [ ] Code blocks display properly
- [ ] Links to exercise files work correctly

### Cross-Reference Validation
- [ ] Links from chapters to glossary work
- [ ] Links from exercises to chapters work
- [ ] All internal links resolve correctly
- [ ] No 404 errors in console

## Expected Output
When running `npm run build`, you should see:
- No build errors
- All new pages included in the build output
- Proper routing for all new content

## Troubleshooting Common Issues

### Markdown Syntax Errors
- Check for proper heading hierarchy
- Verify all code blocks have proper language tags
- Ensure all links have correct paths

### Navigation Issues
- Verify sidebar.js has correct file paths
- Check that all files exist at specified locations
- Ensure file extensions are correct (.md)

### Link Issues
- Verify all relative links use correct paths
- Check that referenced files exist
- Ensure no broken external links

## Automated Validation Script
You can create a simple script to automate these checks:

```bash
#!/bin/bash
# validate-build.sh

echo "Starting Docusaurus build validation..."

# Build the site
echo "Building site..."
npm run build
BUILD_RESULT=$?

if [ $BUILD_RESULT -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
else
    echo "✅ Build successful!"
fi

# Check if all module2 pages are in the build
echo "Checking for module2 content in build..."
if [ -d "build/docs/module2-digital-twin" ]; then
    MODULE2_PAGES=$(ls build/docs/module2-digital-twin/ | wc -l)
    echo "Found $MODULE2_PAGES module2 pages in build"
    if [ $MODULE2_PAGES -ge 7 ]; then
        echo "✅ Module2 content validation passed!"
    else
        echo "❌ Expected at least 7 module2 pages, found $MODULE2_PAGES"
        exit 1
    fi
else
    echo "❌ Module2 directory not found in build!"
    exit 1
fi

echo "🎉 All validation checks passed!"
```

Save this as `validate-build.sh` in the project root and make it executable with `chmod +x validate-build.sh`.

## Continuous Integration Considerations
For integration with CI/CD pipelines:
- Run build validation on every commit
- Include link checking in automated tests
- Set up alerts for broken builds
- Validate content before merging pull requests