"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";

export function GlobalNav() {
  const pathname = usePathname();

  const isActive = (path: string) => {
    if (path === "/") {
      return pathname === "/" || pathname === "/tasks";
    }
    return pathname.startsWith(path);
  };

  return (
    <header className="w-full h-16 bg-white border-b border-gray-200 sticky top-0 z-50">
      <div className="max-w-[1440px] mx-auto px-6 h-full flex items-center justify-between">
        {/* Logo */}
        <Link href="/" className="flex items-center gap-2 cursor-pointer">
          <svg
            className="w-8 h-8 text-black"
            fill="currentColor"
            viewBox="0 0 24 24"
          >
            <path d="M12 2L14.5 9.5L22 12L14.5 14.5L12 22L9.5 14.5L2 12L9.5 9.5L12 2Z" />
          </svg>
          <span className="font-bold text-lg tracking-tight text-black">
            MuJoCo Demo
          </span>
        </Link>

        {/* Navigation Links */}
        <nav className="hidden md:flex items-center gap-1">
          <Link
            href="/tasks"
            className={`px-6 py-2 rounded-full text-sm font-medium transition-colors ${
              isActive("/tasks")
                ? "bg-gray-100 text-black"
                : "text-gray-600 hover:bg-gray-50 hover:text-black"
            }`}
          >
            Tasks
          </Link>
        </nav>

        {/* Right side - can add links if needed */}
        <div className="flex items-center gap-4">
          <a
            href="https://github.com"
            target="_blank"
            rel="noopener noreferrer"
            className="px-4 py-2 rounded-full text-sm font-medium text-gray-600 hover:bg-gray-50 hover:text-black transition-colors"
          >
            GitHub
          </a>
        </div>
      </div>
    </header>
  );
}
