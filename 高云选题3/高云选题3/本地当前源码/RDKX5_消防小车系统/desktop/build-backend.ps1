# Retained as an explicit migration notice for existing build instructions.
# The Windows app now connects only to the real X5 service; it has no local backend.
throw '离线后台打包入口已停用。请运行 desktop/build-app.ps1 -Check 构建真实控制客户端，并在 X5 部署 presentation 服务。'
