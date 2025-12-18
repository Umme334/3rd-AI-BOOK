Skip to main content
[![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)**AI-Native Textbooks**](/textbook-generation/)[Textbooks](/textbook-generation/textbooks/intro)[Blog](/textbook-generation/blog)
[GitHub](https://github.com/your-org/textbook-generation)
On this page
# Manage Docs Versions
Docusaurus can manage multiple versions of your docs.
## Create a docs version​
Release a version 1.0 of your project:
    
    npm run docusaurus docs:version 1.0  
    
The `docs` folder is copied into `versioned_docs/version-1.0` and `versions.json` is created.
Your docs now have 2 versions:
  * `1.0` at `http://localhost:3000/docs/` for the version 1.0 docs
  * `current` at `http://localhost:3000/docs/next/` for the **upcoming, unreleased docs**


## Add a Version Dropdown​
To navigate seamlessly across versions, add a version dropdown.
Modify the `docusaurus.config.js` file:
docusaurus.config.js
    
    export default {  
      themeConfig: {  
        navbar: {  
          items: [  
            {  
              type: 'docsVersionDropdown',  
            },  
          ],  
        },  
      },  
    };  
    
The docs version dropdown appears in your navbar:
![Docs Version Dropdown](/textbook-generation/assets/images/docsVersionDropdown-35e13cbe46c9923327f30a76a90bff3b.png)
## Update an existing version​
It is possible to edit versioned docs in their respective folder:
  * `versioned_docs/version-1.0/hello.md` updates `http://localhost:3000/docs/hello`
  * `docs/hello.md` updates `http://localhost:3000/docs/next/hello`


[Edit this page](https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/tutorial-extras/manage-docs-versions.md)
  * Create a docs version
  * Add a Version Dropdown
  * Update an existing version


Textbooks
  * [Getting Started](/textbook-generation/textbooks/intro)
  * [Physical AI](/textbook-generation/textbooks/physical-ai/introduction)
  * [Humanoid Robotics](/textbook-generation/textbooks/humanoid-robotics/hardware)


More
  * [Blog](/textbook-generation/blog)
  * [GitHub](https://github.com/your-org/textbook-generation)


Copyright © 2025 AI-Native Textbook for Physical AI and Humanoid Robotics. Built with Docusaurus.
