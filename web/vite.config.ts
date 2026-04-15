import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Change ROBOT_HOST to your robot's IP for local development
const ROBOT_HOST = process.env.ROBOT_HOST || 'localhost:5050'

export default defineConfig({
  plugins: [react()],
  server: {
    port: 3000,
    proxy: {
      '/api': `http://${ROBOT_HOST}`,
      '/ws': { target: `ws://${ROBOT_HOST}`, ws: true },
      '/mcp': `http://${ROBOT_HOST}`,
      '/map': `http://${ROBOT_HOST}`,
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
})
