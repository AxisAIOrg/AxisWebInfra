import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  reactStrictMode: true,
  
  // Allow cross-origin requests from 127.0.0.1 during development
  allowedDevOrigins: ["http://127.0.0.1:3000"],
  
  // Turbopack config
  turbopack: {},
  
  // Webpack config (for when using --webpack flag)
  webpack: (config, { isServer, webpack }) => {
    // Handle WASM files
    config.experiments = {
      ...config.experiments,
      asyncWebAssembly: true,
    };

    // Don't try to process mujoco-js on the server side at all
    if (isServer) {
      config.externals = [...(config.externals || []), "mujoco-js"];
      return config;
    }

    // For client side, exclude mujoco-js from normal bundling
    // It will be loaded dynamically at runtime
    config.resolve.alias = {
      ...config.resolve.alias,
    };

    // Completely ignore mujoco-js - don't try to bundle it at all
    // The module will be loaded as a pure ES module at runtime
    config.resolve.fallback = {
      ...config.resolve.fallback,
      fs: false,
      path: false,
      crypto: false,
      stream: false,
      buffer: false,
      util: false,
      assert: false,
      http: false,
      https: false,
      os: false,
      url: false,
      zlib: false,
    };

    // Completely ignore mujoco-js - prevent webpack from analyzing it
    // This prevents webpack from trying to bundle or analyze the mujoco-js module
    const externalsArray = Array.isArray(config.externals) 
      ? [...config.externals] 
      : config.externals 
        ? [config.externals] 
        : [];
    
    config.externals = [
      ...externalsArray,
      function ({ request }: { request: string | undefined }, callback: (err?: Error | null, result?: string) => void) {
        // Exclude mujoco-js from bundling completely
        if (request?.includes("mujoco-js") || request?.includes("/mujoco-js/")) {
          // Return undefined to tell webpack to ignore it completely
          return callback(undefined, undefined);
        }
        callback();
      },
    ];
    
    // Also add to resolve to prevent resolution during build
    config.resolve.alias = {
      ...config.resolve.alias,
      // Don't resolve mujoco-js - let it load at runtime
    };

    // Use IgnorePlugin to completely ignore mujoco-js during compilation
    // This prevents webpack from even trying to resolve or analyze the module
    if (webpack && webpack.IgnorePlugin) {
      config.plugins = [
        ...(config.plugins || []),
        new webpack.IgnorePlugin({
          resourceRegExp: /mujoco-js/,
        }),
      ];
    }

    // Ignore import.meta.url warnings/errors from mujoco-js
    // Don't add rules that process mujoco-js - let it be external

    return config;
  },
};

export default nextConfig;
