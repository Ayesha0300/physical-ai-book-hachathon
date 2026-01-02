import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useLocation } from '@docusaurus/router';

const Navbar = (props) => {
  const location = useLocation();

  // Define the mapping of routes to navigation items
  const getActiveNavItem = () => {
    const pathname = location.pathname;

    if (pathname.startsWith('/modules') || pathname.includes('/docs/modules')) {
      return 'Modules';
    } else if (pathname.startsWith('/capstone') || pathname.includes('/docs/capstone')) {
      return 'Capstone';
    } else if (pathname.startsWith('/hardware') || pathname.includes('/docs/hardware')) {
      return 'Hardware';
    } else if (pathname === '/' || pathname.startsWith('/home')) {
      return 'Home';
    } else {
      return null;
    }
  };

  const activeItem = getActiveNavItem();

  // Clone the original props to modify the items
  const modifiedProps = {
    ...props,
    // Update the logo to use our custom SVG
    logo: {
      ...props.logo,
      src: '/img/logo.svg',
      alt: 'Physical AI Logo',
      height: 32,  // Adjust height as needed
      width: 32,   // Adjust width as needed
    },
    items: props.items?.map(item => {
      // For items that have a 'to' property (internal links)
      if (item.to) {
        const itemName = item.label || (item.to === '/' ? 'Home' : item.to.split('/')[1]);

        // Special case for modules since it's a complex path
        if ((item.to.includes('/modules') || item.label === 'Modules') && activeItem === 'Modules') {
          return { ...item, active: true };
        }

        // Match other items based on their label
        if (item.label === activeItem) {
          return { ...item, active: true };
        }
      }

      // For external links or items without 'to', just return as-is
      return { ...item, active: item.active || false };
    })
  };

  return <OriginalNavbar {...modifiedProps} />;
};

export default Navbar;