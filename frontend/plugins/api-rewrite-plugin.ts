// frontend/plugins/api-rewrite-plugin.ts
import { LoadContext } from '@docusaurus/types';
import * as path from 'path'; // Import path module

interface PluginOptions {
  backendUrl: string;
}

export default function apiRewritePlugin(
  context: LoadContext,
  options: PluginOptions
): any {
  return {
    name: 'api-rewrite-plugin',
    injectHtmlTags(): any {
      const { backendUrl } = options;
      return {
        headTags: [
          {
            tagName: 'script',
            innerHTML: `
              (function() {
                const BACKEND_URL = '${backendUrl}'; // Use the passed backendUrl
                const ORIGINAL_FETCH = window.fetch;

                window.fetch = function(...args) {
                  let [resource, config] = args;

                  if (typeof resource === 'string' && resource.startsWith('/api')) {
                    const newResource = BACKEND_URL + resource.substring(4); // Remove /api and prepend backend URL
                    console.log('Proxying API request:', resource, '->', newResource);
                    return ORIGINAL_FETCH(newResource, config);
                  }

                  return ORIGINAL_FETCH(resource, config);
                };
              })();
            `,
          },
        ],
      };
    },
  };
}
