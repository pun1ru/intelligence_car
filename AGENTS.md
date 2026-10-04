# Agent 工作约定

开始修改前，先通读 `docs/` 下的 Markdown 文档，包括 `docs/TODO_LIST.md` 和 `docs/LOG.md`，了解项目约定、当前目标和已有记录。涉及控制、通信、车体约定或工程结构时，修改过程中再次核对对应文档，不要仅凭代码推断项目约定。

修改代码时遵守 `docs/C_CODE_STYLE.md` 和 `docs/ARCHITECTURE.md`；控制相关改动同时遵守 `docs/CONTROL_DESIGN.md`，协议相关改动同时遵守 `docs/SERIAL_PROTOCOL.md`。保留第三方代码的原有风格，避免无关改动。

完成变更时，同步更新受影响的 `docs/` 文档，使其中的行为、参数和状态与实际实现一致；若完成或新增待办，同步更新 `docs/TODO_LIST.md`。在 `docs/LOG.md` 追加简短日志，写明日期、变更内容、验证结果及未解决事项。未验证的结果要明确标注，不要写成已通过。
