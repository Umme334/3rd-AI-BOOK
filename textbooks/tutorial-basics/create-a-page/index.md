Skip to main content
[![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)**AI-Native Textbooks**](/textbook-generation/)[Textbooks](/textbook-generation/textbooks/intro)[Blog](/textbook-generation/blog)
[GitHub](https://github.com/your-org/textbook-generation)
On this page
# Create a Page
Add **Markdown or React** files to `src/pages` to create a **standalone page** :
  * `src/pages/index.js` → `localhost:3000/`
  * `src/pages/foo.md` → `localhost:3000/foo`
  * `src/pages/foo/bar.js` → `localhost:3000/foo/bar`


## Create your first React Page​
Create a file at `src/pages/my-react-page.js`:
src/pages/my-react-page.js
    
    import React from 'react';  
    import Layout from '@theme/Layout';  
      
    export default function MyReactPage() {  
      return (  
        <Layout>  
          <h1>My React page</h1>  
          <p>This is a React page</p>  
        </Layout>  
      );  
    }  
    
A new page is now available at <http://localhost:3000/my-react-page>.
## Create your first Markdown Page​
Create a file at `src/pages/my-markdown-page.md`:
src/pages/my-markdown-page.md
    
    # My Markdown page  
      
    This is a Markdown page  
    
A new page is now available at <http://localhost:3000/my-markdown-page>.
[Edit this page](https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/tutorial-basics/create-a-page.md)
  * Create your first React Page
  * Create your first Markdown Page


Textbooks
  * [Getting Started](/textbook-generation/textbooks/intro)
  * [Physical AI](/textbook-generation/textbooks/physical-ai/introduction)
  * [Humanoid Robotics](/textbook-generation/textbooks/humanoid-robotics/hardware)


More
  * [Blog](/textbook-generation/blog)
  * [GitHub](https://github.com/your-org/textbook-generation)


Copyright © 2025 AI-Native Textbook for Physical AI and Humanoid Robotics. Built with Docusaurus.
