import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";
import { GlobalNav } from "@/components/GlobalNav";
import { GlobalFooter } from "@/components/GlobalFooter";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "MuJoCo Robot Control Demo",
  description: "Open-source robot teleoperation demo using MuJoCo physics simulation",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="bg-white">
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased bg-white`}
        suppressHydrationWarning={true}
      >
        <div className="flex min-h-screen flex-col bg-white">
          <GlobalNav />
          <main className="flex-1 bg-white">{children}</main>
          <GlobalFooter />
        </div>
      </body>
    </html>
  );
}
