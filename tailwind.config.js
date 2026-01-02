module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './static/**/*.{js,jsx,ts,tsx}',
  ],
  theme: {
    extend: {
      colors: {
        'physical-ai-navy': '#1e3a8a',      // Dark navy for primary
        'physical-ai-charcoal': '#2d3748',  // Charcoal for secondary
        'physical-ai-blue': '#06b6d4',      // Electric blue for accent
        'physical-ai-cyan': '#0ea5e9',      // Cyan for secondary accent
      },
    },
  },
  plugins: [],
};