import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    port: 3000,
    proxy: {
      '/api': 'http://localhost:5050',
      '/ws': { target: 'ws://localhost:5050', ws: true },
      '/mcp': 'http://localhost:5050',
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
})
