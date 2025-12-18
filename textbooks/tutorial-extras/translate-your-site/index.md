Skip to main content
[![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)![Physical AI and Humanoid Robotics Logo](/textbook-generation/img/logo.svg)**AI-Native Textbooks**](/textbook-generation/)[Textbooks](/textbook-generation/textbooks/intro)[Blog](/textbook-generation/blog)
[GitHub](https://github.com/your-org/textbook-generation)
On this page
# Translate your site
Let's translate `docs/intro.md` to French.
## Configure i18n​
Modify `docusaurus.config.js` to add support for the `fr` locale:
docusaurus.config.js
    
    export default {  
      i18n: {  
        defaultLocale: 'en',  
        locales: ['en', 'fr'],  
      },  
    };  
    
## Translate a doc​
Copy the `docs/intro.md` file to the `i18n/fr` folder:
    
    mkdir -p i18n/fr/docusaurus-plugin-content-docs/current/  
      
    cp docs/intro.md i18n/fr/docusaurus-plugin-content-docs/current/intro.md  
    
Translate `i18n/fr/docusaurus-plugin-content-docs/current/intro.md` in French.
## Start your localized site​
Start your site on the French locale:
    
    npm run start -- --locale fr  
    
Your localized site is accessible at <http://localhost:3000/fr/> and the `Getting Started` page is translated.
caution
In development, you can only use one locale at a time.
## Add a Locale Dropdown​
To navigate seamlessly across languages, add a locale dropdown.
Modify the `docusaurus.config.js` file:
docusaurus.config.js
    
    export default {  
      themeConfig: {  
        navbar: {  
          items: [  
            {  
              type: 'localeDropdown',  
            },  
          ],  
        },  
      },  
    };  
    
The locale dropdown now appears in your navbar:
![Locale Dropdown](/textbook-generation/assets/images/localeDropdown-f0d995e751e7656a1b0dbbc1134e49c2.png)
## Build your localized site​
Build your site for a specific locale:
    
    npm run build -- --locale fr  
    
Or build your site to include all the locales at once:
    
    npm run build  
    
[Edit this page](https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/tutorial-extras/translate-your-site.md)
  * Configure i18n
  * Translate a doc
  * Start your localized site
  * Add a Locale Dropdown
  * Build your localized site


Textbooks
  * [Getting Started](/textbook-generation/textbooks/intro)
  * [Physical AI](/textbook-generation/textbooks/physical-ai/introduction)
  * [Humanoid Robotics](/textbook-generation/textbooks/humanoid-robotics/hardware)


More
  * [Blog](/textbook-generation/blog)
  * [GitHub](https://github.com/your-org/textbook-generation)


Copyright © 2025 AI-Native Textbook for Physical AI and Humanoid Robotics. Built with Docusaurus.
