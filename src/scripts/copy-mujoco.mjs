import { cp, mkdir } from "node:fs/promises";
import { dirname, resolve } from "node:path";

const source = resolve("node_modules/mujoco-js/dist");
const destination = resolve("public/mujoco-js/dist");
await mkdir(dirname(destination), { recursive: true });
await cp(source, destination, { recursive: true });
