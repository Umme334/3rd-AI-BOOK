Skip to main content
[![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)**AI-Native Textbooks**](/textbook-generation/)[Textbooks](/textbook-generation/textbooks/intro)[Blog](/textbook-generation/blog)
[GitHub](https://github.com/your-org/textbook-generation)
On this page
# Create a Document
Documents are **groups of pages** connected through:
  * a **sidebar**
  * **previous/next navigation**
  * **versioning**


## Create your first Doc​
Create a Markdown file at `docs/hello.md`:
docs/hello.md
    
    # Hello  
      
    This is my **first Docusaurus document**!  
    
A new document is now available at <http://localhost:3000/docs/hello>.
## Configure the Sidebar​
Docusaurus automatically **creates a sidebar** from the `docs` folder.
Add metadata to customize the sidebar label and position:
docs/hello.md
    
    ---  
    sidebar_label: 'Hi!'  
    sidebar_position: 3  
    ---  
      
    # Hello  
      
    This is my **first Docusaurus document**!  
    
It is also possible to create your sidebar explicitly in `sidebars.js`:
sidebars.js
    
    export default {  
      tutorialSidebar: [  
        'intro',  
        'hello',  
        {  
          type: 'category',  
          label: 'Tutorial',  
          items: ['tutorial-basics/create-a-document'],  
        },  
      ],  
    };  
    
[Edit this page](https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/tutorial-basics/create-a-document.md)
  * Create your first Doc
  * Configure the Sidebar


Textbooks
  * [Getting Started](/textbook-generation/textbooks/intro)
  * [Physical AI](/textbook-generation/textbooks/physical-ai/introduction)
  * [Humanoid Robotics](/textbook-generation/textbooks/humanoid-robotics/hardware)


More
  * [Blog](/textbook-generation/blog)
  * [GitHub](https://github.com/your-org/textbook-generation)


Copyright © 2025 AI-Native Textbook for Physical AI and Humanoid Robotics. Built with Docusaurus.
