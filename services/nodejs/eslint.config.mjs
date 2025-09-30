// Flat config for ESLint v9+
import js from '@eslint/js';
import pluginImport from 'eslint-plugin-import';
import pluginPromise from 'eslint-plugin-promise';
import pluginN from 'eslint-plugin-n';

export default [
  {
    ignores: ['eslint.config.*', 'node_modules/**', 'package-lock.json'],
  },
  {
    files: ['**/*.js'],
    languageOptions: {
      ecmaVersion: 'latest',
      sourceType: 'commonjs',
      globals: {
        console: 'readonly',
        process: 'readonly',
        __dirname: 'readonly',
        module: 'readonly',
        require: 'readonly',
        setInterval: 'readonly',
        clearInterval: 'readonly',
      },
    },
    plugins: {
      import: pluginImport,
      promise: pluginPromise,
      n: pluginN,
    },
    rules: {
      ...js.configs.recommended.rules,
      ...pluginImport.configs.recommended.rules,
      ...pluginPromise.configs.recommended.rules,
      ...pluginN.configs['flat/recommended'].rules,
      'no-unused-vars': ['warn', { args: 'after-used', argsIgnorePattern: '^_' }],
      'no-console': 'off',
      'n/hashbang': 'off',
      'n/no-process-exit': 'off',
      'n/no-missing-require': 'off',
    },
  },
];


