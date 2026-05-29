# Contributing to robotops_msgs

Thanks for your interest in contributing to `robotops_msgs`!

Please open issues or pull requests on the project repository. Keep changes focused, update documentation when behavior changes, and run the relevant build or verification commands from the Docker-based development environment described in [README.md](README.md).

## Use of Generative AI Tools

Robot Ops permits contributions — code, documentation, and other content — that are produced in whole or in part using generative AI tools (e.g. GitHub Copilot, Claude, Cursor, ChatGPT). However, any use of such tools must be disclosed at the time of contribution, recorded in the commit message and pull request description. Contributors are responsible for reviewing, testing, and taking full ownership of all submitted content regardless of how it was produced — AI-generated output should receive the same (or greater) scrutiny as hand-written code, including checks for correctness, security, and intellectual property issues. This policy is aligned with the [OSRF Generative AI Contributions Policy](https://osralliance.org/wp-content/uploads/2025/05/OSRF-Policy-on-the-Use-of-Generative-Tools-Generative-AI-in-Contributions.pdf) (May 2025).

**Commit message disclosure format:**

```text
Fix race condition in topic subscriber lifecycle management

Resolves an issue where subscribers were not correctly unregistered
on node shutdown under high message frequency. Adds a guard to the
teardown sequence and extends the relevant unit tests.

Closes #187.

Generated-by: Claude (Anthropic, claude-sonnet-4-5, May 2025)
```

## License

By contributing, you agree that your contributions will be licensed under the Apache 2.0 license.
