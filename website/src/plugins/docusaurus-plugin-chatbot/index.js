const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './src/ChatbotInjector')];
    },

    configureWebpack(config, isServer) {
      return {
        resolve: {
          alias: {
            '@site/src/components/chatbot': path.resolve(__dirname, '../../src/components/chatbot'),
          },
        },
      };
    },
  };
};