-- Neovim project-specific LSP configuration (Nvim 0.11+)
-- This file is automatically loaded by exrc when placed at project root

vim.lsp.config('rust_analyzer', {
	settings = {
		['rust-analyzer'] = {
			cargo = {
				allTargets = false,
				target = 'thumbv7em-none-eabihf',
			},
			check = {
				allTargets = false,
				command = 'check',
			},
		},
	},
})

vim.lsp.enable('rust_analyzer')
