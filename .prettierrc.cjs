// Resolve from this config, including pre-commit's NODE_PATH.
// Markdown and YAML hooks do not install the XML plugin or use its override.
let xmlPlugin = "@prettier/plugin-xml";
try {
  xmlPlugin = require.resolve(xmlPlugin, { paths: [__dirname] });
} catch {
  // Prettier only loads the plugin when the XML override matches.
}

module.exports = {
  overrides: [
    {
      files: [
        "*.{mission,rviz,tbsdrn,yaml,yml}",
        ".clang-format",
        ".clang-tidy",
        ".cmake-format",
      ],
      options: {
        parser: "yaml",
        printWidth: 120,
        tabWidth: 2,
        useTabs: false,
      },
    },
    {
      files: [
        "*.{dae,policy,qrc,sdf,svg,tplxml,uadf,ui,urdf,world,xacro,xml}",
        "model.config",
      ],
      options: {
        parser: "xml",
        plugins: [xmlPlugin],
        printWidth: 120,
        tabWidth: 2,
        useTabs: false,
        xmlSelfClosingSpace: true,
        xmlWhitespaceSensitivity: "ignore",
      },
    },
    {
      files: "*.{dae,svg,ui}",
      options: {
        xmlWhitespaceSensitivity: "preserve",
      },
    },
  ],
};
