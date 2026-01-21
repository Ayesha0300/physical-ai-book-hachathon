import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { MDXProvider } from '@mdx-js/react';
import FloatingChatbot from '../components/Chatbot/FloatingChatbot';

// Import custom components that should be available globally
import CodeBlock from '../components/CodeBlock';
import CalloutBox from '../components/CalloutBox';
import GlossaryTooltip from '../components/GlossaryTooltip';
import CollapsibleSection from '../components/CollapsibleSection';
import SectionAnchor from '../components/SectionAnchor';
import LoadingSpinner from '../components/LoadingSpinner';
import VisualSystemDiagram from '../components/VisualSystemDiagram';
import CapstoneLayout from '../components/CapstoneLayout';

// Define the mapping of custom components
const components = {
  CodeBlock,
  CalloutBox,
  GlossaryTooltip,
  CollapsibleSection,
  SectionAnchor,
  LoadingSpinner,
  VisualSystemDiagram,
  CapstoneLayout,
  // Shortcuts for common callout types
  Note: (props) => <CalloutBox type="note" {...props} />,
  Warning: (props) => <CalloutBox type="warning" {...props} />,
  Concept: (props) => <CalloutBox type="concept" {...props} />,
  Anchor: (props) => <SectionAnchor {...props} />
};

function Layout(props) {
  return (
    <MDXProvider components={components}>
      <OriginalLayout {...props} />
      <FloatingChatbot />
    </MDXProvider>
  );
}

export default Layout;