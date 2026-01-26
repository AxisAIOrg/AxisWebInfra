"use client";

import Link from "next/link";

export function GlobalFooter() {
  return (
    <footer className="w-full bg-gray-50 pt-16 pb-8 border-t border-gray-200">
      <div className="max-w-[1440px] mx-auto px-6">
        {/* Top Section: Brand + Social */}
        <div className="flex flex-col md:flex-row justify-between items-start mb-12">
          <div className="max-w-md">
            <div className="flex items-center gap-2 mb-4">
              <svg
                className="w-8 h-8 text-black"
                fill="currentColor"
                viewBox="0 0 24 24"
              >
                <path d="M12 2L14.5 9.5L22 12L14.5 14.5L12 22L9.5 14.5L2 12L9.5 9.5L12 2Z" />
              </svg>
              <span className="font-bold text-xl">MuJoCo Demo</span>
            </div>
            <p className="text-gray-600 text-sm leading-relaxed">
              Open-source robot teleoperation demo using MuJoCo physics simulation.
              Control a Franka Emika Panda robotic arm with keyboard controls.
            </p>
          </div>
          <div className="flex gap-4 mt-6 md:mt-0">
            {/* GitHub Link */}
            <a
              href="https://github.com"
              target="_blank"
              rel="noopener noreferrer"
              className="w-8 h-8 bg-white border border-gray-200 flex items-center justify-center rounded hover:bg-gray-100 transition-colors"
              title="View on GitHub"
            >
              <svg
                className="w-5 h-5 text-black"
                fill="currentColor"
                viewBox="0 0 24 24"
              >
                <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.05-.015-2.055-3.33.72-4.035-1.605-4.035-1.605-.54-1.38-1.335-1.755-1.335-1.755-1.095-.75.075-.735.075-.735 1.215.09 1.845 1.245 1.845 1.245 1.08 1.845 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.225 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405 1.02 0 2.04.135 3 .405 2.295-1.56 3.3-1.23 3.3-1.23.66 1.695.24 2.925.12 3.225.765.84 1.23 1.92 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.285 0 .315.225.69.825.57A12.02 12.02 0 0024 12c0-6.63-5.37-12-12-12z" />
              </svg>
            </a>
          </div>
        </div>

        {/* Links Grid */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-8 border-b border-gray-200 pb-12 mb-8">
          <div className="flex flex-col gap-3">
            <h4 className="font-bold text-gray-900 mb-1">Navigation</h4>
            <Link
              href="/tasks"
              className="text-sm text-gray-500 hover:text-black"
            >
              Tasks
            </Link>
          </div>
          <div className="flex flex-col gap-3">
            <h4 className="font-bold text-gray-900 mb-1">Resources</h4>
            <a
              href="https://mujoco.org/"
              target="_blank"
              rel="noopener noreferrer"
              className="text-sm text-gray-500 hover:text-black"
            >
              MuJoCo Documentation
            </a>
            <a
              href="https://github.com"
              target="_blank"
              rel="noopener noreferrer"
              className="text-sm text-gray-500 hover:text-black"
            >
              GitHub Repository
            </a>
          </div>
          <div className="flex flex-col gap-3">
            <h4 className="font-bold text-gray-900 mb-1">Technology</h4>
            <span className="text-sm text-gray-500">Next.js</span>
            <span className="text-sm text-gray-500">Three.js</span>
            <span className="text-sm text-gray-500">MuJoCo</span>
          </div>
          <div className="flex flex-col gap-3">
            <h4 className="font-bold text-gray-900 mb-1">License</h4>
            <span className="text-sm text-gray-500">MIT License</span>
          </div>
        </div>

        {/* Bottom Section: Copyright */}
        <div className="flex flex-col md:flex-row justify-between items-center gap-4">
          <div className="text-xs text-gray-500">
            <span>Open Source Robot Control Demo</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="relative flex h-2.5 w-2.5">
              <span className="relative inline-flex rounded-full h-2.5 w-2.5 bg-green-500"></span>
            </span>
            <span className="text-xs font-mono font-medium text-green-600">
              Local Mode
            </span>
          </div>
        </div>
      </div>
    </footer>
  );
}
