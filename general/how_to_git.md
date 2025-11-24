# Git Policies for GEICar Project
Policies and guidelines for using Git in the GEICar project. For those unfamiliar with Git, please check out the [Git Workflow](#git-workflow) section at the bottom of this document.

## Branching Strategy
- `main` : stable branch for release-ready code.
- `develop` : continuous integration branch for ongoing development. All features are merged here first before merging into `main`.
- `feature/<feature-name>` : branches for developing new features. Created from `develop` and merged back into `develop` when complete.
- `hotfix/<hotfix-name>` : branches for urgent fixes to the `main` or `develop` branches. Created from the branch that needs the fix and merged back into both `main` and `develop`.

## Commit Message Conventions
Follow the Conventional Commits specification:
```
<type>[optional scope]: <description>
```

Where `<type>` can be:
- `feat` : a new feature
- `fix` : a bug fix
- `docs` : documentation changes
- `refactor` : code changes that neither fix a bug nor add a feature
- `test` : adding or updating tests

For `[optional scope]`, specify the affected area (e.g., `nav`, `arm`, `hmi`, `ai`, etc).

Please provide a brief but descriptive `<description>` of the change for each commit.

## Pull Request Guidelines
- All merges into `develop` and `main` must be done via Pull Requests (PRs).
- PRs must be reviewed and approved by at least one, preferably two, team members before merging.

To see how to make a pull request, refer to the [Creating a Pull Request](https://github.blog/developer-skills/github/beginners-guide-to-github-creating-a-pull-request/) guide.

## Useful VSCode Extensions
- GitGraph: Visualize your repository and see code changes.

## Git Workflow
### Installation
#### Windows
1. Download and install Git from [git-scm.com](https://git-scm.com/download/win).
2. Follow the installation prompts and choose default settings unless you have specific preferences.
3. After installation, open Command Prompt or PowerShell and run `git --version` to verify the installation.
4. Configure your Git username and email:
   ```
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```
5. Create a GitHub account if you don't have one already at [github.com](https://github.com).
6. GitHub will ask you for your email and password when you push changes for the first time.

> CAUTION: On Windows; git commants may ONLY work in Git Bash, you can open Git Bash by right-clicking in any folder and selecting "Git Bash Here".

### Initializing a Repository
1. Navigate to your project directory
2. To initialize a new Git repository, run:
```
git init
```
3. Create a `.gitignore` file to specify files and directories to be ignored by Git. See further down for a sample `.gitignore` file.

### Cloning a Repository
To clone an existing repository from GitHub, navigate to the desired project, click the "Code" button, copy the URL in the HTTPS tab, and run:
```
git clone <repository-url>
```

### Basic Git Commands

#### Before coding
- Fetch and pull the latest changes from the remote repository:
```
git pull origin <branch-name>
```
- Create and switch to a new branch for your feature or fix:
```
git checkout -b feature/<feature-name>
```
- Verify the status of your working directory:
```
git status
```
- Check the commit history:
```
git log
```

#### After coding
- Stage changes for commit:
```
git add <file1> <file2> ...
```
Or to stage all changes:
```
git add .
```
- Commit your changes with a descriptive message:
```
git commit -m "message"
```
- Push your branch to the remote repository:
```
git push origin feature/<feature-name>
```
- To merge your changes into another branch, first switch to that branch:
```
git checkout branch_to_merge_into
```
Then merge your feature branch:
```
git merge feature/<feature-name>
```
This should not be done directly to `main` or `develop` branches; use Pull Requests instead.

- To revert changes in your working directory see [Undoing Changes](https://www.atlassian.com/git/tutorials/undoing-changes)
### Handling Conflicts
When merging branches, conflicts may arise if the same lines of code have been modified in both branches. To resolve conflicts, open VSCode, which highlights conflicting sections. Manually edit the files to resolve conflicts, then stage and commit the resolved files.
Check out this guide for more details: [Resolve a merge conflict using VSCode](https://dev.to/adiatiayu/how-to-resolve-merge-conflicts-using-the-merge-editor-feature-on-vs-code-pic).