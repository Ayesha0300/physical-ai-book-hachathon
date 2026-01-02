import React from 'react';
import OriginalFooter from '@theme-original/Footer';
import { useLocation } from '@docusaurus/router';

const Footer = (props) => {
  const location = useLocation();

  // Enhance the footer items to ensure they point to valid pages
  const enhancedProps = {
    ...props,
    links: props.links?.map((linkSection, sectionIndex) => {
      return {
        ...linkSection,
        items: linkSection.items?.map(item => {
          // Correct any potentially broken links
          if (item.to) {
            let correctedTo = item.to;

            // Fix common broken links
            if (correctedTo.includes('/docs/modules/README')) {
              correctedTo = '/modules';
            } else if (correctedTo.includes('/docs/capstone/README')) {
              correctedTo = '/capstone';
            } else if (correctedTo.includes('/docs/hardware/README')) {
              correctedTo = '/hardware';
            } else if (correctedTo.includes('/docs/intro')) {
              correctedTo = '/';
            } else if (correctedTo.includes('/docs/glossary')) {
              correctedTo = '/'; // Redirect to home since glossary may not exist yet
            } else if (correctedTo.includes('/docs/credits')) {
              correctedTo = '/'; // Redirect to home since credits may not exist yet
            }

            return { ...item, to: correctedTo };
          }

          return item;
        })
      };
    })
  };

  return <OriginalFooter {...enhancedProps} />;
};

export default Footer;